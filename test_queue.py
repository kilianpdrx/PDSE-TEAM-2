from flask import Flask, Response, jsonify
import threading
import time
import cv2
from queue import Queue
from ai_camera import IMX500Detector
import json
import numpy as np

# Initialisation du modèle et de la caméra
model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)

# Démarrage de la caméra
camera.start(show_preview=True)
time.sleep(1)  # Laisser le temps à la caméra de démarrer


# Configuration Flask
app = Flask(__name__)

# Queue pour les frames et leurs données associées
frame_queue = Queue(maxsize=10)

# Variables de configuration
center_x, center_y = 320, 240  # Centre de l'image pour le calcul
w, h = 640, 480               # Dimensions de l'image




# Fonction pour convertir récursivement les float32 en float
def convert_floats(data):
    if isinstance(data, dict):
        return {key: convert_floats(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [convert_floats(item) for item in data]
    elif isinstance(data, np.float32):  # Si c'est un float32, le convertir en float
        return float(data)
    else:
        return data





# Thread pour capturer les frames et leurs données
def capture_frames_and_data():
    while True:
        if not frame_queue.full():
            # Capture d'une image
            frame = camera.picam2.capture_array()
            
            # Détection des objets
            detections = camera.get_detections()
            labels = camera.get_labels()
            
            # Rassembler les données de détection
            detection_data = {
                "timestamp": time.time(),
                "detections": []
            }

            for d in detections:
                x, y, w, h = d.box
                detection_data['detections'].append({
                    "label": labels[int(d.category)],
                    "confidence": d.conf,
                    "bbox": [x, y, w, h]
                })


            detection_data = convert_floats(detection_data)
            # print(detection_data)   
            # Mettre la frame et ses données dans la queue
            frame_queue.put((frame, detection_data))
        
        time.sleep(0.001)  # Capture ~33 FPS

# Route combinée pour le flux vidéo et les détections
@app.route('/combined_feed')
def combined_feed():
    def generate_combined_feed():
        while True:
            if not frame_queue.empty():
                frame, data = frame_queue.get()  # Récupérer la frame et ses données depuis la queue

                # Convertir l'image en JPEG
                _, buffer = cv2.imencode('.jpg', frame)
                img_data = buffer.tobytes()
                
                # Convertir les données de détection en JSON
                detection_data = json.dumps(data)
                print(detection_data)
                
                # Combiner les deux (image + données de détection) dans un seul flux
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + img_data + b'\r\n'
                       b'--data\r\n'
                       b'Content-Type: application/json\r\n\r\n' + detection_data.encode() + b'\r\n')
            time.sleep(0.001)
    
    return Response(generate_combined_feed(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Lancement des threads
if __name__ == "__main__":
    # Lancer les threads de capture des frames et des données
    threading.Thread(target=capture_frames_and_data, daemon=True).start()

    # Démarrer le serveur Flask
    app.run(host='0.0.0.0', port=5000)
