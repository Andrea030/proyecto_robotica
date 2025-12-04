from machine import UART, Pin
import time
import random
import json

# --- Configuración UART ---
# TX del Pico -> RX del Arduino
# RX del Pico -> TX del Arduino (si quieres recibir algo)
uart = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))  # Ajusta los pines según tu conexión

# Función para generar valores entre -1 y 1
def generar_valores():
    x = round(random.uniform(-1, 1), 3)
    y = round(random.uniform(-1, 1), 3)
    return x, y

# Función para enviar datos en formato compatible con tu Arduino
def enviar_valores_uart(x, y):
    mensaje = f"{x},{y}\n"
    uart.write(mensaje.encode())
    print("Enviado pro:", mensaje)
    print("Enviado:", mensaje.strip())

# --- Bucle principal ---
while True:
    # Generar dos sets de valores
    x1, y1 = generar_valores()
    x2, y2 = generar_valores()
    
    # Enviar el primer set
    enviar_valores_uart(x1, y1)
    time.sleep(3)
    
    # Enviar el segundo set
    enviar_valores_uart(x2, y2)
    time.sleep(3)



