import sensor
import image
import time
import pyb
from pyb import Pin, Timer

# --------------------------------
# Configuración de luz PWM
# --------------------------------
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))

TARGET_BRIGHTNESS = 35
MIN_PWM = 5
MAX_PWM = 100
KP = 1
ALPHA = 0.15
pwm = 50
light.pulse_width_percent(pwm)

# --------------------------------
# Configuración de cámara
# --------------------------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_exposure(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# --------------------------------
# UART
# --------------------------------
uart = pyb.UART(3, 115200)

# --------------------------------
# ROIs
# --------------------------------
roi  = (20, 140, 280, 10)
roi1 = (20, 140, 140, 10)   # izquierda
roi2 = (160, 140, 140, 10)  # derecha

# --------------------------------
# Thresholds
# --------------------------------
red_threshold   = (30, 85, 15, 70, -10, 55)
green_threshold = (30, 90, -128, -10, 0, 127)

color = 0

def buscar_colores(img, ROI, color_rect=(255, 255, 255)):
    img.draw_rectangle(ROI, color=color_rect)

    area_rojo  = 0
    area_verde = 0
    area_mayor = 0

    red_blobs = img.find_blobs([red_threshold],
                               roi=ROI,
                               pixels_threshold=200,
                               area_threshold=200,
                               merge=True)
    for blob in red_blobs:
        img.draw_rectangle(blob.rect(), color=(255, 0, 0))
        a = blob.w() * blob.h()
        area_rojo = a
        if a > area_mayor:
            area_mayor = a

    green_blobs = img.find_blobs([green_threshold],
                                 roi=ROI,
                                 pixels_threshold=200,
                                 area_threshold=200,
                                 merge=True)
    for blob in green_blobs:
        img.draw_rectangle(blob.rect(), color=(0, 255, 0))
        a = blob.w() * blob.h()
        area_verde = a
        if a > area_mayor:
            area_mayor = a

    return area_rojo, area_verde, area_mayor


def determinar_color(ar1, av1, ar2, av2):
    area_roja_total  = ar1 + ar2
    area_verde_total = av1 + av2

    if area_roja_total > 0 and area_verde_total == 0:
        return 1
    elif area_verde_total > 0 and area_roja_total == 0:
        return 2
    elif area_roja_total > 0 and area_verde_total > 0:
        return 1 if area_roja_total > area_verde_total else 2
    else:
        return 0


def determinar_posicion(area_mayor_1, area_mayor_2):
    if area_mayor_1 == 0 and area_mayor_2 == 0:
        return 0
    if area_mayor_1 >= area_mayor_2:
        return 1  # izquierda
    else:
        return 2  # derecha


while True:
    clock.tick()
    img = sensor.snapshot()

    # --------------------------------
    # Control automático de luz (PID suavizado)
    # --------------------------------
    stats = img.get_statistics(roi=(100, 80, 120, 80))
    brightness = stats.l_mean()
    error = TARGET_BRIGHTNESS - brightness
    desired_pwm = pwm + (KP * error)
    desired_pwm = max(MIN_PWM, min(MAX_PWM, desired_pwm))
    pwm = (1 - ALPHA) * pwm + ALPHA * desired_pwm
    light.pulse_width_percent(int(pwm))

    # Buscar colores en ambos ROIs
    ar1, av1, mayor1 = buscar_colores(img, roi1, color_rect=(0, 255, 255))
    ar2, av2, mayor2 = buscar_colores(img, roi2, color_rect=(255, 0, 255))

    # Color detectado
    color = determinar_color(ar1, av1, ar2, av2)

    # Posición del blob más grande
    posicion = determinar_posicion(mayor1, mayor2)

    # --------------------------------
    # Decisión, print y UART
    # --------------------------------
    if color == 1:
        print("Rojo | Pos:", posicion, "| Brillo:", brightness, "PWM:", int(pwm), "FPS:", clock.fps())
    elif color == 2:
        print("Verde | Pos:", posicion, "| Brillo:", brightness, "PWM:", int(pwm), "FPS:", clock.fps())
    else:
        print("Nada | Brillo:", brightness, "PWM:", int(pwm), "FPS:", clock.fps())
        posicion = 0

    uart.write("C:%d,P:%d\n" % (color, posicion))
