# ESP32 - MicroPython
# TF-Luna LiDAR x3 + Recepción UART desde cámara OpenMV
# Sensor 1: SDA=GPIO21, SCL=GPIO22
# Sensor 2: SDA=GPIO13, SCL=GPIO15
# Sensor 3: SDA=GPIO27, SCL=GPIO26
# UART cámara: RX=GPIO16, TX=GPIO17

from machine import Pin, SoftI2C, UART
import time

# ── Buses I2C ─────────────────────────────────────────
i2c1 = SoftI2C(scl=Pin(22), sda=Pin(21))   # Sensor TF-Luna 1
i2c2 = SoftI2C(scl=Pin(15), sda=Pin(13))   # Sensor TF-Luna 2
i2c3 = SoftI2C(scl=Pin(26), sda=Pin(27))   # Sensor TF-Luna 3

TF_LUNA_ADDR = 0x10

# ── UART para recibir datos de la cámara OpenMV ───────
uart = UART(2, baudrate=115200, rx=16, tx=17)

# ── Verificar conexión al iniciar ──────────────────────
print("Escaneando I2C1 (GPIO21/22):", i2c1.scan())
print("Escaneando I2C2 (GPIO13/15):", i2c2.scan())
print("Escaneando I2C3 (GPIO27/26):", i2c3.scan())
print("=" * 50)

# Variables cámara
color    = 0
posicion = 0


# ── Función: leer distancia de un TF-Luna ─────────────
def leer_distancia(bus):
    try:
        bus.writeto(TF_LUNA_ADDR, bytes([0x00]))
        buf = bus.readfrom(TF_LUNA_ADDR, 6)
        distancia = (buf[1] << 8) | buf[0]
        return distancia
    except OSError:
        return None


# ── Función: leer y parsear datos UART de la cámara ───
# La cámara envía: "C:1,P:2\n"
def leer_camara():
    global color, posicion
    if uart.any():
        try:
            linea = uart.readline().decode("utf-8").strip()
            # Parsear "C:1,P:2"
            partes = linea.split(",")
            color    = int(partes[0].split(":")[1])
            posicion = int(partes[1].split(":")[1])
        except:
            pass  # Si hay error en el parsing, conservar último valor


# ── Función: texto descriptivo del color ──────────────
def nombre_color(c):
    if c == 1: return "Rojo"
    if c == 2: return "Verde"
    return "Nada"


# ── Función: texto descriptivo de la posición ─────────
def nombre_posicion(p):
    if p == 1: return "Izquierda"
    if p == 2: return "Derecha"
    return "Sin posicion"


# ── Loop principal ─────────────────────────────────────
while True:

    # Leer cámara
    leer_camara()

    # Leer sensores LiDAR
    valor1 = leer_distancia(i2c1)
    valor2 = leer_distancia(i2c2)
    valor3 = leer_distancia(i2c3)

    # ── Imprimir datos cámara ──────────────────────────
    print(f"[Camara]   Color: {nombre_color(color)} | Posicion: {nombre_posicion(posicion)}")

    # ── Imprimir datos LiDAR ───────────────────────────
    if valor1 is None or valor1 > 1000:
        print(f"[Sensor 1] Lectura cancelada: {valor1}")
    else:
        print(f"[Sensor 1] Distancia: {valor1} cm")

    if valor2 is None or valor2 > 1000:
        print(f"[Sensor 2] Lectura cancelada: {valor2}")
    else:
        print(f"[Sensor 2] Distancia: {valor2} cm")

    if valor3 is None or valor3 > 1000:
        print(f"[Sensor 3] Lectura cancelada: {valor3}")
    else:
        print(f"[Sensor 3] Distancia: {valor3} cm")

    print("-" * 50)
    time.sleep(0.1)