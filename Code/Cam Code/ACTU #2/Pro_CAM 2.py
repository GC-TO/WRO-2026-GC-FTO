import sensor
import image
import time
import pyb
import gc
from pyb import Pin, Timer

# =========================================================
# CONFIGURACION GENERAL
# =========================================================
DEBUG = False          # Cambiar a True solo para pruebas con computadora
DRAW_DEBUG = False     # Cambiar a True solo si quieres ver rectangulos en OpenMV IDE

LED_RED = pyb.LED(1)
LED_GREEN = pyb.LED(2)
LED_BLUE = pyb.LED(3)

# =========================================================
# ARRANQUE AUTONOMO SEGURO
# =========================================================
LED_RED.on()
time.sleep_ms(300)
LED_RED.off()

# Espera para que ESP32 / alimentacion se estabilicen
time.sleep_ms(3000)

# =========================================================
# UART HACIA ESP32
# OpenMV P4 TX -> ESP32 GPIO4 RX
# GND comun
# =========================================================
uart = pyb.UART(3, 115200, timeout_char=20)

# Enviar estado seguro al iniciar
for i in range(10):
    uart.write("C:0,P:0\n")
    time.sleep_ms(50)

LED_GREEN.on()
time.sleep_ms(200)
LED_GREEN.off()

# =========================================================
# CONFIGURACION DE LUZ PWM VARIABLE, LIMITADA AL 60%
# =========================================================
# Para LED no hace falta 50 kHz. 500 Hz es mas estable.
light = Timer(2, freq=500).channel(1, Timer.PWM, pin=Pin("P6"))

TARGET_BRIGHTNESS = 60

MIN_PWM = 5
MAX_PWM = 60      # Limite maximo: nunca pasa de 60%

KP = 0.6
ALPHA = 0.20

pwm = 10

# Arranque suave para evitar pico de corriente
light.pulse_width_percent(0)
time.sleep_ms(500)

for p in range(0, pwm + 1):
    light.pulse_width_percent(p)
    time.sleep_ms(80)

# =========================================================
# CONFIGURACION DE CAMARA
# =========================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)

# QQVGA = 160 x 120
sensor.set_framesize(sensor.QQVGA)

# Bloquear automaticos para estabilidad de color
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)

sensor.skip_frames(time=2000)

clock = time.clock()

LED_BLUE.on()
time.sleep_ms(200)
LED_BLUE.off()

# =========================================================
# ROIS PARA QQVGA
# =========================================================
# Imagen QQVGA: 160 x 120
# Franja de deteccion inferior
ROI_LEFT = (10, 70, 70, 8)
ROI_RIGHT = (80, 70, 70, 8)

# ROI para medir brillo general
ROI_BRIGHTNESS = (50, 40, 60, 40)

# =========================================================
# THRESHOLDS DE COLOR
# =========================================================
RED_THRESHOLD = (30, 85, 15, 70, -10, 55)
GREEN_THRESHOLD = (30, 90, -128, -10, 0, 127)

# En QQVGA las areas son menores que en QVGA
PIXELS_THRESHOLD = 50
AREA_THRESHOLD = 50
MIN_TOTAL_AREA = 70

# =========================================================
# TIEMPOS
# =========================================================
SEND_INTERVAL_MS = 80
FORCE_SEND_MS = 250
PRINT_INTERVAL_MS = 800
LED_INTERVAL_MS = 300
GC_INTERVAL_MS = 2000

last_send = time.ticks_ms()
last_force_send = time.ticks_ms()
last_print = time.ticks_ms()
last_led = time.ticks_ms()
last_gc = time.ticks_ms()

last_color = -1
last_position = -1

# =========================================================
# FUNCIONES
# =========================================================
def clamp(value, min_value, max_value):
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def update_light(img):
    global pwm

    try:
        stats = img.get_statistics(roi=ROI_BRIGHTNESS)
        brightness = stats.l_mean()

        error = TARGET_BRIGHTNESS - brightness

        desired_pwm = pwm + (KP * error)
        desired_pwm = clamp(desired_pwm, MIN_PWM, MAX_PWM)

        pwm = (1 - ALPHA) * pwm + ALPHA * desired_pwm
        pwm = clamp(pwm, MIN_PWM, MAX_PWM)

        light.pulse_width_percent(int(pwm))

        return brightness

    except Exception:
        # Si falla la lectura de brillo, dejar luz en valor seguro
        pwm = MIN_PWM
        light.pulse_width_percent(int(pwm))
        return -1


def get_area(img, roi, threshold, draw_color=None):
    total_area = 0

    try:
        blobs = img.find_blobs(
            [threshold],
            roi=roi,
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True
        )

        for blob in blobs:
            area = blob.w() * blob.h()
            total_area += area

            if DRAW_DEBUG and draw_color is not None:
                img.draw_rectangle(blob.rect(), color=draw_color)

    except Exception:
        total_area = 0

    return total_area


def detect_color_position(img):
    if DRAW_DEBUG:
        img.draw_rectangle(ROI_LEFT, color=(0, 255, 255))
        img.draw_rectangle(ROI_RIGHT, color=(255, 0, 255))

    # Areas por color y lado
    left_red = get_area(img, ROI_LEFT, RED_THRESHOLD, (255, 0, 0))
    right_red = get_area(img, ROI_RIGHT, RED_THRESHOLD, (255, 0, 0))

    left_green = get_area(img, ROI_LEFT, GREEN_THRESHOLD, (0, 255, 0))
    right_green = get_area(img, ROI_RIGHT, GREEN_THRESHOLD, (0, 255, 0))

    total_red = left_red + right_red
    total_green = left_green + right_green

    # Filtro contra ruido
    if total_red < MIN_TOTAL_AREA and total_green < MIN_TOTAL_AREA:
        return 0, 0, total_red, total_green

    # Color dominante y posicion del mismo color
    if total_red >= total_green:
        color = 1  # rojo

        if left_red >= right_red:
            position = 1
        else:
            position = 2

    else:
        color = 2  # verde

        if left_green >= right_green:
            position = 1
        else:
            position = 2

    return color, position, total_red, total_green


def send_uart(color, position, force=False):
    global last_color, last_position, last_send, last_force_send

    now = time.ticks_ms()

    changed = (color != last_color) or (position != last_position)
    send_time_ok = time.ticks_diff(now, last_send) >= SEND_INTERVAL_MS
    force_time_ok = time.ticks_diff(now, last_force_send) >= FORCE_SEND_MS

    if send_time_ok and (changed or force or force_time_ok):
        uart.write("C:%d,P:%d\n" % (color, position))

        last_color = color
        last_position = position
        last_send = now

        if force or force_time_ok:
            last_force_send = now


def heartbeat():
    global last_led

    now = time.ticks_ms()

    if time.ticks_diff(now, last_led) >= LED_INTERVAL_MS:
        LED_GREEN.toggle()
        last_led = now


def clean_memory():
    global last_gc

    now = time.ticks_ms()

    if time.ticks_diff(now, last_gc) >= GC_INTERVAL_MS:
        gc.collect()
        last_gc = now


def debug_print(color, position, red_area, green_area, brightness):
    global last_print

    if not DEBUG:
        return

    now = time.ticks_ms()

    if time.ticks_diff(now, last_print) >= PRINT_INTERVAL_MS:
        if color == 1:
            color_text = "Rojo"
        elif color == 2:
            color_text = "Verde"
        else:
            color_text = "Nada"

        print(
            color_text,
            "| Pos:", position,
            "| Red:", red_area,
            "| Green:", green_area,
            "| Brillo:", brightness,
            "| PWM:", int(pwm),
            "| FPS:", clock.fps(),
            "| Mem:", gc.mem_free()
        )

        last_print = now


# =========================================================
# LOOP PRINCIPAL
# =========================================================
print("OpenMV lista en QQVGA con PWM limitado al 60%.")

while True:
    try:
        clock.tick()

        img = sensor.snapshot()

        # Actualizar intensidad LED sin pasar de 60%
        brightness = update_light(img)

        # Detectar color y posicion
        color, position, red_area, green_area = detect_color_position(img)

        # Enviar a ESP32
        send_uart(color, position)

        # Mantenimiento
        heartbeat()
        clean_memory()

        # Debug opcional
        debug_print(color, position, red_area, green_area, brightness)

    except Exception as e:
        # Estado seguro si algo falla
        uart.write("C:0,P:0\n")

        LED_RED.on()
        time.sleep_ms(80)
        LED_RED.off()

        if DEBUG:
            print("Error loop:", e)

        time.sleep_ms(50)
