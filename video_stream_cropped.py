from flask import Flask, Response
import time
from ai_camera import IMX500Detector
import cv2
import uuid
from queue import Queue
import threading
import json


# Initialisation du modèle et de la caméra
model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)

center_x=320
center_y=240
w=640
h=480

# Configuration Flask
app = Flask(__name__)

# Démarrage de la caméra avec aperçu activé
camera.start(show_preview=True)

frame_queue = Queue(maxsize=10)

# Associe une URL (/video_feed) à une fonction Python
# Quand un utilisateur accède à http://<ip>:5000/video_feed, la fonction video_feed est exécutée.


# Thread pour capturer les frames et leurs données
def capture_frames_and_data():
    while True:
        if not frame_queue.full():
            # Capture d'une image
            frame = camera.picam2.capture_array()
            
            # Détection des objets
            detections = camera.get_detections()
            labels = camera.get_labels()

            # Rassembler les données de détecltion
            detection_data = {
                "detections": []
            }


            for detection in detections:
                if int(detection.category) > len(labels) - 1:
                    continue
                
                label = labels[int(detection.category)]
                if label != "person":  # Filtre uniquement pour les personnes
                    continue
                
                x, y, w, h = detection.box
                detection_data['detections'].append({
                    "label": labels[int(detection.category)],
                    "bbox": [x, y, w, h]
                })
        
         


            # print(detection_data)   
            # Mettre la frame et ses données dans la queue
            frame_queue.put((frame, detection_data))
        
        time.sleep(0.001)  # Capture ~33 FPS

# Route pour le flux vidéo complet
@app.route('/full_feed')
def video_feed():
    def generate_full_frames():
        try:
            while True:
                frame, data = frame_queue.get()

                # Encodage de l'image complète
                _, buffer = cv2.imencode('.jpg', frame)

                # Envoi de l'image
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                time.sleep(0.001)  # Petite pause pour réduire la charge

        except GeneratorExit:
            print("Client déconnecté (full feed).")
        finally:
            camera.stop()
            print("Arrêt de la caméra (full feed).")
            exit()

    return Response(generate_full_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/cropped_feed')
def cropped_feed():
    def generate_cropped_frames():
        try:
            while True:
                frame, data = frame_queue.get()

                detections = camera.get_detections()
                labels = camera.get_labels()

                for detection in detections:
                    if int(detection.category) > len(labels) - 1:
                        continue
                    
                    label = labels[int(detection.category)]
                    if label != "person":  # Filtre uniquement pour les personnes
                        continue

                    # Découpe de la personne détectée
                    x, y, w, h = detection.box
                    label_txt = f"{labels[int(detection.category)]} ({detection.conf:.2f})"

    
                    text_x = x + 5
                    text_y = y + 15 


                    cv2.putText(frame, label_txt, (text_x, text_y),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0, 0), thickness=2)


                    mid_x = int(x + w / 2)
                    mid_y = int(y + h / 2)
                    
                    
                    cv2.circle(frame, (mid_x,mid_y), 5, (255,0,0), -1)  # Dessiner un cercle au centre de la boîte
                    cv2.line(frame, (center_x, center_y), (mid_x,mid_y), (0,0,255), 2)  # Ligne de trajectoire (rouge)
                
                    cropped_person = frame[y:y+h, x:x+w]

                    # Générer un identifiant unique
                    image_id = uuid.uuid4().hex

                    # Encodage de l'image découpée
                    _, buffer = cv2.imencode('.jpg', cropped_person)

                    # Envoi de l'image avec un identifiant unique
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n' +
                           f'X-Image-ID: {image_id}\r\n\r\n'.encode('utf-8') +
                           buffer.tobytes() + b'\r\n')

                time.sleep(0.001)  # Petite pause pour réduire la charge

        except GeneratorExit:
            print("Client déconnecté (cropped feed).")
        finally:
            camera.stop()
            print("Arrêt de la caméra (cropped feed).")
            exit()

    return Response(generate_cropped_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')





# faire ce data la en flot continu 
# si possible le synchroniser avec le flux video
@app.route('/data')
def send_data():
    def generate_data():
        while True:
            if not frame_queue.empty():
                _, data = frame_queue.get()
                yield f"data: {json.dumps(data)}\n\n"
            time.sleep(0.03)  # Correspond au taux de frame (33 FPS)

    return Response(generate_data(), content_type='text/event-stream')


if __name__ == "__main__":

    threading.Thread(target=capture_frames_and_data, daemon=True).start()

    app.run(host='0.0.0.0', port=5000)