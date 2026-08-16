from machine import UART, Pin, SoftI2C
from time import sleep_ms, ticks_ms, ticks_diff
from pupremote import PUPRemoteSensor, SPIKE_ULTRASONIC
import gc


# =========================================================
# CONFIGURACION GENERAL
# =========================================================
DEBUG = False          # True solo para pruebas con Thonny
PRINT_MS = 500         # Imprimir cada 500 ms si DEBUG = True

# PUPRemote debe atenderse con frecuencia mientras negocia con el Hub.
CONNECTION_POLL_MS = 2

# Tiempos de trabajo
DIST_SEND_MS = 20
CAM_SEND_MS = 40
TF_READ_GAP_MS = 6
TF_WAIT_MS = 3

CAM_TIMEOUT_MS = 250
DIST_TIMEOUT_MS = 300


# =========================================================
# CONFIGURACION TF-LUNA
# =========================================================
TF_ADDR = 0x10
CMD_GET_MM = b'\x5A\x05\x00\x06\x65'
I2C_FREQ = 400000

i2c_1 = SoftI2C(scl=Pin(21), sda=Pin(22), freq=I2C_FREQ)
i2c_2 = SoftI2C(scl=Pin(26), sda=Pin(27), freq=I2C_FREQ)
i2c_3 = SoftI2C(scl=Pin(15), sda=Pin(13), freq=I2C_FREQ)
i2c_buses = (i2c_1, i2c_2, i2c_3)


# =========================================================
# UART DESDE OPENMV
# OpenMV P4 TX -> ESP32 GPIO4 RX
# GND comun
# Formato: C:x,P:y\n
# =========================================================
uart = UART(1, baudrate=115200, rx=Pin(4), timeout=0)
buffer = bytearray()


# =========================================================
# PUPREMOTE HACIA HUB LEGO
# 16 bytes: configuracion recomendada para Pybricks.
# =========================================================
pr = PUPRemoteSensor(
    sensor_id=SPIKE_ULTRASONIC,
    power=True,
    max_packet_size=16,
)

# Mantener exactamente los mismos nombres, formatos y orden en el Hub.
pr.add_channel("cam", "h")
pr.add_channel("dist", "hhh")

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
    """Lee los tres TF-Luna por turnos sin bloquear el bucle principal."""
    global tf_estado, tf_cmd_ms, tf_index, ultimo_tf_read

    bus = i2c_buses[tf_index]

    if tf_estado == TF_ESTADO_LIBRE:
        if ticks_diff(now, ultimo_tf_read) < TF_READ_GAP_MS:
            return

        try:
            bus.writeto(TF_ADDR, CMD_GET_MM)
            tf_cmd_ms = now
            tf_estado = TF_ESTADO_ESPERANDO
        except Exception as error:
            marcar_timeout_si_corresponde(tf_index, now)
            tf_index = (tf_index + 1) % 3
            ultimo_tf_read = now

            if DEBUG:
                print("Error comando TF-Luna:", tf_index, error)
        return

    if ticks_diff(now, tf_cmd_ms) < TF_WAIT_MS:
        return

    try:
        data = bus.readfrom(TF_ADDR, 9)

        if len(data) == 9 and data[0] == 0x59 and data[1] == 0x59:
            checksum = (
                data[0] + data[1] + data[2] + data[3]
                + data[4] + data[5] + data[6] + data[7]
            ) & 0xFF

            if checksum == data[8]:
                distancia = data[2] | (data[3] << 8)
                distancias[tf_index] = limitar_int16(distancia)
                ultimo_ok_dist[tf_index] = now
            else:
                marcar_timeout_si_corresponde(tf_index, now)
        else:
            marcar_timeout_si_corresponde(tf_index, now)

    except Exception as error:
        marcar_timeout_si_corresponde(tf_index, now)

        if DEBUG:
            print("Error lectura TF-Luna:", tf_index, error)

    tf_index = (tf_index + 1) % 3
    ultimo_tf_read = now
    tf_estado = TF_ESTADO_LIBRE


def procesar_linea_openmv(linea):
    """Convierte b'C:1,P:2' en 0, 11, 12, 21 o 22."""
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

        if len(c_part) != 3 or len(p_part) != 3:
            return None

        color = c_part[2] - 0x30
        posicion = p_part[2] - 0x30

        if color not in (0, 1, 2) or posicion not in (0, 1, 2):
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

    if (
        valor_cam != ultimo_valor_cam_enviado
        or ticks_diff(now, ultimo_envio_cam) >= CAM_SEND_MS
    ):
        pr.update_channel("cam", valor_cam)
        ultimo_valor_cam_enviado = valor_cam
        ultimo_envio_cam = now


def restaurar_canales():
    """Publica nuevamente los ultimos valores tras una conexion/reconexion."""
    pr.update_channel("cam", valor_cam)
    pr.update_channel("dist", distancias[0], distancias[1], distancias[2])


def conectar_hub():
    """
    Mantiene PUPRemote activo hasta completar la negociacion con el Hub.
    Llama process() una sola vez por iteracion.
    """
    while True:
        try:
            if bool(pr.process()):
                return True
        except Exception as error:
            if DEBUG:
                print("Error conectando con Hub:", error)

        sleep_ms(CONNECTION_POLL_MS)


def debug_print(now):
    global ultimo_print

    if DEBUG and ticks_diff(now, ultimo_print) >= PRINT_MS:
        print(
            "CAM:", valor_cam,
            "| D1:", distancias[0],
            "D2:", distancias[1],
            "D3:", distancias[2],
        )
        ultimo_print = now


def limpieza_memoria(now):
    global ultimo_gc

    if ticks_diff(now, ultimo_gc) >= 2000:
        gc.collect()
        ultimo_gc = now


# =========================================================
# CONEXION INICIAL CON EL HUB
# =========================================================
if DEBUG:
    print("ESP32 lista: OpenMV + 3 TF-Luna + PUPRemote")
    print("Esperando conexion con el Hub...")

gc.collect()
hub_conectado = conectar_hub()
restaurar_canales()

if DEBUG:
    print("Hub conectado. Entrando al bucle principal.")


# =========================================================
# LOOP PRINCIPAL
# =========================================================
while True:
    try:
        now = ticks_ms()

        try:
            hub_conectado = bool(pr.process())
        except Exception as error:
            hub_conectado = False

            if DEBUG:
                print("Error PUPRemote:", error)

        if not hub_conectado:
            if DEBUG:
                print("Conexion perdida. Reconectando...")

            hub_conectado = conectar_hub()
            restaurar_canales()

            if DEBUG:
                print("Hub reconectado.")

            # Reiniciar referencias de tiempo despues de una espera larga.
            now = ticks_ms()
            ultimo_tf_read = now
            ultimo_envio_dist = now
            ultimo_envio_cam = now
            ultimo_rx_cam = now
            tf_estado = TF_ESTADO_LIBRE
            continue

        # Camara
        leer_openmv()
        actualizar_stale_camara(now)
        enviar_camara(now)

        # Distancias
        tick_tfluna(now)
        enviar_distancias(now)

        # Mantenimiento
        debug_print(now)
        limpieza_memoria(now)

        sleep_ms(1)

    except Exception as error:
        if DEBUG:
            print("Error general:", error)

        sleep_ms(10)