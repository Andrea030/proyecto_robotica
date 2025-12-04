import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
import os
import time

# --- Librerías de BallTracker ---
from ultralytics import YOLO
import pandas as pd

# --- Configuración MQTT ---
BROKER = "172.17.144.248"  
PORT = 1883
TOPIC = "robot/pico/estado"  

# Intento de conexión MQTT
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
try:
    client.connect(BROKER, PORT, 60)
    client.loop_start() 
    print(f"Conectado al broker MQTT en {BROKER}:{PORT}")
except Exception as e:
    print(f"Error al conectar con el broker MQTT: {e}")

# --- Clase de Rastreo de Pelota (CONFIDENCIA AJUSTADA) ---
class BallTracker:
    """Clase para la detección de la pelota usando un modelo YOLO."""

    def __init__(self, model_path):
        # Inicializa el modelo YOLO
        self.model = YOLO(model_path)
        print(f"Modelo YOLO cargado desde: {model_path}")

    def detect_frame(self, frame):
        """Detecta la pelota en un solo frame."""
        # conf=0.30 (Umbral de confianza aumentado para acelerar un poco)
        # verbose=False
        results = self.model.predict(frame, conf=0.30, verbose=False, imgsz=320)[0] 

        ball_dict = {}
        for box in results.boxes:
            result = box.xyxy.tolist()[0]
            ball_dict[1] = result 
            break
            
        return ball_dict

# --- Configuración del Modelo YOLO Ball Tracker ---
BALL_MODEL_PATH = 'models/best.pt'
try:
    ball_tracker = BallTracker(model_path=BALL_MODEL_PATH)
except Exception as e:
    print(f"ERROR CRÍTICO: No se pudo inicializar BallTracker con {BALL_MODEL_PATH}. Error: {e}")
    exit()

# --- Captura de cámara (RESOLUCIÓN REDUCIDA) ---
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
# >> AÑADE ESTAS LÍNEAS PARA REDUCIR LA CARGA DE CÁLCULO <<
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Error: no se pudo abrir la cámara")
    exit()

cv2.namedWindow("Detección en Cámara", cv2.WINDOW_NORMAL)
print("Iniciando detección. Presiona 'q' para salir...")

frame_count = 0
PUB_EVERY_N_FRAMES = 2
coordenadas = ""
# --- Bucle de Procesamiento en Tiempo Real ---
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: no se pudo leer el frame de la cámara")
        break

    (h, w) = frame.shape[:2]
    
    # Detección de BallTracker (Pelota YOLO)
    ball_dict = ball_tracker.detect_frame(frame)
    
    # Procesamiento y Publicación MQTT (Solo Pelota)
    if ball_dict:
        # Obtener BBox de la pelota
        bbox = ball_dict[1]
        x1, y1, x2, y2 = [int(x) for x in bbox]

        # Dibujar BBox de la Pelota (Color Amarillo)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
        cv2.putText(frame, "Ball (YOLO)", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        
        # Calcular centro para MQTT
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        coordenadas = ""
        if frame_count % PUB_EVERY_N_FRAMES == 0:
            ball_msg = json.dumps({
                "objeto": "ball",
                "x_center": round(center_x / w, 4), 
                "y_center": round(center_y / h, 4),
                "confidence": "N/A"
            })
            coordenadas = f"x: {round((center_x / w)-0.5, 4)}, y = {abs(round((center_y / h)-1, 4))}"
            print(coordenadas)
            try:
                client.publish(TOPIC, ball_msg)
                # print(f"MQTT enviado a {TOPIC} (Ball): {ball_msg}") # Descomenta para ver logs
            except Exception as e:
                print(f"Error al publicar MQTT (Ball): {e}")

    # Mostrar frame
    cv2.imshow("Detección en Cámara", frame)
    frame_count += 1
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Limpieza al salir ---
cap.release()
cv2.destroyAllWindows()
client.loop_stop()
client.disconnect()
print("Desconectado del broker MQTT y cámara liberada.")
