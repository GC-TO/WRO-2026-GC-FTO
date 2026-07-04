from machine import UART, Pin, SoftI2C
from time import sleep_ms, ticks_ms, ticks_diff
from pupremote import PUPRemoteSensor, SPIKE_ULTRASONIC
import gc

# =========================================================
# CONFIGURACION GENERAL
# =========================================================
DEBUG = False          # Cambiar a True solo para pruebas con Thonny
PRINT_MS = 500         # Imprimir cada 500 ms si DEBUG = True

# Tiempos de trabajo
DIST_SEND_MS = 20      # Enviar distancias al hub cada 20 ms
CAM_SEND_MS = 40       # Refrescar valor de cámara cada 40 ms
TF_READ_GAP_MS = 6     # Tiempo entre iniciar lectura de un TF-Luna y el siguiente
TF_WAIT_MS = 3         # Espera entre comando y lectura de datos del TF-Luna

CAM_TIMEOUT_MS = 250   # Si OpenMV no manda datos, cámara vuelve a 0
DIST_TIMEOUT_MS = 300  # Si un TF-Luna no responde, distancia vuelve a 0

# =========================================================
# CONFIGURACION TF-LUNA
# =========================================================
TF_ADDR = 0x10
CMD_GET_MM = b'\x5A\x05\x00\x06\x65'

# Recomendación:
# Si hay cables largos o lecturas inestables, baja a 100000.
I2C_FREQ = 400000

i2c_1 = SoftI2C(scl=Pin(21), sda=Pin(22), freq=I2C_FREQ)
i2c_2 = SoftI2C(scl=Pin(26), sda=Pin(27), freq=I2C_FREQ)
i2c_3 = SoftI2C(scl=Pin(15), sda=Pin(13), freq=I2C_FREQ)

i2c_buses = (i2c_1, i2c_2, i2c_3)

# =========================================================
# UART DESDE OPENMV
# OpenMV P4 TX -> ESP32 GPIO4 RX
# GND común
# Formato esperado desde OpenMV:
# C:x,P:y
# Ejemplo:
# C:1,P:2  -> rojo derecha -> 12
# C:2,P:1  -> verde izquierda -> 21
# =========================================================
uart = UART(1, baudrate=115200, rx=Pin(4), timeout=0)
buffer = bytearray()   # OPTIMIZADO: bytearray en vez de bytes -> evita
                        # crear un objeto nuevo cada vez que llega un chunk

# =========================================================
# PUPREMOTE HACIA HUB LEGO
# =========================================================
pr = PUPRemoteSensor(sensor_id=SPIKE_ULTRASONIC, power=True)

# Canal cámara:
# 0  = nada
# 11 = rojo izquierda
# 12 = rojo derecha
# 21 = verde izquierda
# 22 = verde derecha
pr.add_channel("cam", "h")

# Canal distancias: tres int16
pr.add_channel("dist", "hhh")

# Valores iniciales
pr.update_channel("cam", 0)
pr.update_channel("dist", 0, 0, 0)

# =========================================================
# VARIABLES
# =========================================================
valor_cam = 0
ultimo_valor_cam_enviado = -1

distancias = [0, 0, 0]
ultimo_ok_dist = [ticks_ms(), ticks_ms(), ticks_ms()]

tf_index = 0

# OPTIMIZADO: máquina de estados para el TF-Luna en curso, en vez de
# bloquear el loop con sleep_ms() entre comando y lectura.
TF_ESTADO_LIBRE = 0
TF_ESTADO_ESPERANDO = 1
tf_estado = TF_ESTADO_LIBRE
tf_cmd_ms = ticks_ms()

ultimo_tf_read = ticks_ms()
ultimo_envio_dist = ticks_ms()
ultimo_envio_cam = ticks_ms()
ultimo_print = ticks_ms()
ultimo_gc = ticks_ms()
ultimo_rx_cam = ticks_ms()


# =========================================================
# FUNCIONES
# =========================================================
def limitar_int16(valor):
    if valor < 0:
        return 0
    if valor > 32767:
        return 32767
    return valor


def marcar_timeout_si_corresponde(idx, now):
    if ticks_diff(now, ultimo_ok_dist[idx]) > DIST_TIMEOUT_MS:
        distancias[idx] = 0


def tick_tfluna(now):
    """
    Reemplaza a leer_un_tfluna(). No bloquea nunca el loop:
    - Estado LIBRE: si ya pasó TF_READ_GAP_MS, manda el comando al
      sensor actual y pasa a ESPERANDO.
    - Estado ESPERANDO: si ya pasó TF_WAIT_MS desde el comando, intenta
      leer los 9 bytes de respuesta (sin sleep) y avanza al siguiente
      sensor.
    """
    global tf_estado, tf_cmd_ms, tf_index, ultimo_tf_read

    bus = i2c_buses[tf_index]

    if tf_estado == TF_ESTADO_LIBRE:
        if ticks_diff(now, ultimo_tf_read) < TF_READ_GAP_MS:
            return

        try:
            bus.writeto(TF_ADDR, CMD_GET_MM)
            tf_cmd_ms = now
            tf_estado = TF_ESTADO_ESPERANDO
        except Exception:
            # No se pudo ni mandar el comando: se revisa timeout y se
            # pasa al siguiente sensor sin bloquear.
            marcar_timeout_si_corresponde(tf_index, now)
            tf_index = (tf_index + 1) % 3
            ultimo_tf_read = now
        return

    # tf_estado == TF_ESTADO_ESPERANDO
    if ticks_diff(now, tf_cmd_ms) < TF_WAIT_MS:
        return

    try:
        data = bus.readfrom(TF_ADDR, 9)

        if (len(data) == 9 and data[0] == 0x59 and data[1] == 0x59):
            # OPTIMIZADO: suma directa en vez de sum(data[0:8]),
            # evita crear una copia (slice) del buffer en cada lectura.
            checksum = (data[0] + data[1] + data[2] + data[3] +
                        data[4] + data[5] + data[6] + data[7]) & 0xFF

            if checksum == data[8]:
                distancia = data[2] | (data[3] << 8)
                distancias[tf_index] = limitar_int16(distancia)
                ultimo_ok_dist[tf_index] = now
            else:
                marcar_timeout_si_corresponde(tf_index, now)
        else:
            marcar_timeout_si_corresponde(tf_index, now)

    except Exception:
        marcar_timeout_si_corresponde(tf_index, now)

    tf_index = (tf_index + 1) % 3
    ultimo_tf_read = now
    tf_estado = TF_ESTADO_LIBRE


def procesar_linea_openmv(linea):
    """
    Recibe una línea tipo:
    b'C:1,P:2'
    Retorna:
    0, 11, 12, 21 o 22

    OPTIMIZADO: trabaja directo sobre bytes (sin decode ni doble split),
    baja bastante la basura generada cuando OpenMV manda muchas líneas.
    """
    try:
        if not linea:
            return None

        idx_coma = linea.find(b",")
        if idx_coma < 0:
            return None

        c_part = linea[:idx_coma]
        p_part = linea[idx_coma + 1:]

        if not (c_part.startswith(b"C:") and p_part.startswith(b"P:")):
            return None

        # color y posicion son siempre un solo digito ASCII, asi que
        # se leen directo sin int()/decode(): en MicroPython indexar
        # un byte ya da su valor numerico, solo hay que restar el
        # codigo ASCII de '0' (0x30). Ademas es mas rapido que int().
        if len(c_part) != 3 or len(p_part) != 3:
            return None

        color = c_part[2] - 0x30
        posicion = p_part[2] - 0x30

        # Valores válidos:
        # color: 0 nada, 1 rojo, 2 verde
        # posicion: 0 nada, 1 izquierda, 2 derecha
        if color not in (0, 1, 2):
            return None

        if posicion not in (0, 1, 2):
            return None

        if color == 0 or posicion == 0:
            return 0

        return color * 10 + posicion

    except Exception:
        return None


def leer_openmv():
    global valor_cam, ultimo_rx_cam

    if uart.any() <= 0:
        return

    data = uart.read()

    if not data:
        return

    buffer.extend(data)

    # Evita que el buffer crezca sin control si llega basura sin salto de línea
    # NOTA: se usa slice-assignment en vez de del, porque no todos los
    # builds de MicroPython soportan "del bytearray[...]".
    if len(buffer) > 80:
        buffer[:] = buffer[-40:]

    while True:
        idx = buffer.find(b"\n")
        if idx < 0:
            break

        linea = bytes(buffer[:idx]).strip()
        buffer[0:idx + 1] = b""

        nuevo_valor = procesar_linea_openmv(linea)

        if nuevo_valor is not None:
            valor_cam = nuevo_valor
            ultimo_rx_cam = ticks_ms()


def actualizar_stale_camara(now):
    global valor_cam

    if ticks_diff(now, ultimo_rx_cam) > CAM_TIMEOUT_MS:
        valor_cam = 0


def enviar_distancias(now):
    global ultimo_envio_dist

    if ticks_diff(now, ultimo_envio_dist) >= DIST_SEND_MS:
        pr.update_channel("dist", distancias[0], distancias[1], distancias[2])
        ultimo_envio_dist = now


def enviar_camara(now):
    global ultimo_envio_cam, ultimo_valor_cam_enviado

    # Enviar si cambió o si ya pasó el tiempo de refresco
    if valor_cam != ultimo_valor_cam_enviado or ticks_diff(now, ultimo_envio_cam) >= CAM_SEND_MS:
        pr.update_channel("cam", valor_cam)
        ultimo_valor_cam_enviado = valor_cam
        ultimo_envio_cam = now


def debug_print(now):
    global ultimo_print

    if not DEBUG:
        return

    if ticks_diff(now, ultimo_print) >= PRINT_MS:
        print(
            "CAM:", valor_cam,
            "| D1:", distancias[0],
            "D2:", distancias[1],
            "D3:", distancias[2]
        )
        ultimo_print = now


def limpieza_memoria(now):
    global ultimo_gc

    if ticks_diff(now, ultimo_gc) >= 2000:
        gc.collect()
        ultimo_gc = now


# =========================================================
# LOOP PRINCIPAL
# =========================================================
print("ESP32 lista: OpenMV + 3 TF-Luna + PUPRemote (optimizado, sin bloqueos I2C)")

while True:
    try:
        now = ticks_ms()

        # Lo más importante: atender PUPRemote lo más seguido posible
        pr.process()

        # Cámara
        leer_openmv()
        actualizar_stale_camara(now)
        enviar_camara(now)

        # Distancias: máquina de estados no bloqueante, un paso por vuelta
        tick_tfluna(now)
        enviar_distancias(now)

        # Debug y mantenimiento
        debug_print(now)
        limpieza_memoria(now)

        # Pausa muy corta para no ahogar el procesador
        sleep_ms(1)

    except Exception as e:
        if DEBUG:
            print("Error general:", e)

        # Evita que un error deje congelado el sistema
        sleep_ms(10)