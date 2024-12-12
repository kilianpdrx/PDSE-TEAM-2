from flask import Flask, Response, request, jsonify
import time
import cv2
import uuid
from queue import Queue
import threading
import json
import serial
import math

from ai_camera import IMX500Detector



center_x=320
center_y=240
# w=640
# h=480

DELAY = 0.01
# HUMAN_SIZE = 1.87
CONVERSION_FACTOR = 0.00024

model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"




def calculate_angle(xdist, depth):

    # Vérification des entrées
    if depth == 0:
        raise ValueError("La profondeur ne peut pas être nulle pour éviter une division par zéro.")
    
    # Calcul de l'angle
    mdist = CONVERSION_FACTOR * xdist
    angle = math.atan2(mdist, depth)
    return angle



def calculate_distance(human_size, height_box):

    if height_box <= 0:
        raise ValueError("La valeur en pixels doit être strictement positive.")
    
    # fonction affine pour la conversion de pixels en mètres
    coeff = -0.02
    ordo = 6.3

    # Calcul de la distance A VERIFIER
    distance = coeff * (height_box/human_size) + ordo
    return distance






class Serveur:
    def __init__(self):
        self.app = Flask(__name__)
        
        self.camera = IMX500Detector(model)
        self.camera.start()
        
        self.arduino = serial.Serial('/dev/ttyACM0', 9600)
        
        self.HUMAN_SIZE = 1.87
        
        
        self.frame_queue = Queue(maxsize=10)
        self.receiver_queue = Queue(maxsize=10)
        
        
        
        
        # Route pour le flux vidéo complet
        @self.app.route('/full_feed')
        def video_feed(self):
            def generate_full_frames():
                try:
                    while True:
                        frame = self.camera.picam2.capture_array()

                        # Encodage de l'image complète
                        _, buffer = cv2.imencode('.jpg', frame)

                        # Envoi de l'image
                        yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                        time.sleep(DELAY)  # Petite pause pour réduire la charge

                except GeneratorExit:
                    print("Client déconnecté (full feed).")
                finally:
                    self.camera.stop()
                    print("Arrêt de la caméra (full feed).")
                    exit()

            return Response(generate_full_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


            
        @self.app.route('/cropped_feed')
        def cropped_feed(self):
            def generate_cropped_frames():
                try:
                    while True:
                        if not self.frame_queue.empty():
                            cropped_images, _ = self.frame_queue.get()

                            for cropped in cropped_images:
                                # Générer un identifiant unique
                                image_id = uuid.uuid4().hex

                                # Envoi de l'image avec un identifiant unique
                                yield (b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n' +
                                    f'X-Image-ID: {image_id}\r\n\r\n'.encode('utf-8') +
                                    cropped + b'\r\n')

                        time.sleep(DELAY)  # Petite pause pour réduire la charge

                except GeneratorExit:
                    print("Client déconnecté (cropped feed).")
                finally:
                    self.camera.stop()
                    print("Arrêt de la caméra (cropped feed).")
                    exit()

            return Response(generate_cropped_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


        @self.app.route('/fusion')
        def fusion(self):
            def generate():
                try:
                    while True:
                        if not self.frame_queue.empty():
                            cropped_frames, extra_data = self.frame_queue.get()  # Récupérer frame et valeur supplémentaire

                            for cropped in cropped_frames:
                                # Générer un identifiant unique
                                image_id = uuid.uuid4().hex


                                # Récupérer la valeur spécifique à envoyer
                                detections = extra_data.get("detections", [])
                                for detection in detections:
                                    bbox = detection.get("bbox", [])
                                    x, y, w, h = bbox

                                    val_send = x+w/2 - self.center_x

                                # Ajouter la valeur en tant qu'en-tête personnalisé
                                yield (b'--frame\r\n'
                                    b'Content-Type: image/jpeg\r\n' +
                                    f'X-bbox: {h}\r\n'.encode('utf-8') +
                                    f'X-dist_x: {val_send}\r\n'.encode('utf-8') +
                                    b'\r\n' +
                                    cropped + b'\r\n')
                                
                        time.sleep(DELAY)
                except GeneratorExit:
                    print("Client déconnecté.")
                except Exception as e:
                    print(f"Erreur: {e}")
                finally:
                    self.camera.stop()
                    print("Arrêt de la caméra.")
                    exit()

            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')



        # faire ce data la en flot continu 
        # si possible le synchroniser avec le flux video
        @self.app.route('/data')
        def send_data(self):
            def generate_data():
                while True:
                    if not self.frame_queue.empty():
                        _, detection_data = self.frame_queue.get()
                        detection_data['id'] = uuid.uuid4().hex  # Ajouter un identifiant unique
                        yield f"data: {json.dumps(detection_data)}\n\n"
                    time.sleep(DELAY)

            return Response(generate_data(), content_type='text/event-stream')



        @self.app.route('/update_data', methods=['POST'])
        def update_data(self):
            live = time.time()
            try:
                # Lire les données JSON envoyées
                data = request.json
                if 'x_distance' in data and 'height_box' in data and "tracking" in data:
                    # print(f"Distance en pixels : {data['x_distance']}")
                    x_distance = float(data['x_distance'])  # Conversion explicite en int
                    height_box = int(data['height_box'])  # Conversion explicite en int   
                    tracking = int(data["tracking"])

                    dist = calculate_distance(self.HUMAN_SIZE, abs(height_box))
                    print(f"Distance calculée : {dist:.2f} mètres")

                    angle = calculate_angle(x_distance, dist)
                    print(f"Angle calculé : {math.degrees(angle):.2f} degrés")

                    self.ardu_talk(dist, angle, tracking)
                    print("Temps de réponse : ", time.time() - live)

                # receiver_queue.put(data)

                return jsonify({"status": "success", "message": "Données reçues avec succès."}), 200
            
            except ValueError:
                # Gestion de l'erreur de conversion en entier
                return jsonify({"status": "error", "message": "'x_distance' doit être convertible en entier"}), 400
            
            except Exception as e:
                # Gestion d'autres erreurs
                return jsonify({"status": "error", "message": str(e)}), 500
            
            
    
    
    def ardu_talk(self, dist, angle):
        """
        Envoie des valeurs de distance et d'angle à une Arduino via une connexion série.

        Args:
            ser (serial.Serial): Instance de la connexion série.
            dist (float): Distance calculée en mètres.
            angle (float): Angle calculé en radians.
        """
        try:
            # Convertir les données en une chaîne formatée pour l'Arduino
            message = f"{dist:.2f},{math.degrees(angle):.2f}\n"  # Format : "distance,angle\n"
            
            # Envoyer le message sur le port série
            self.arduino.write(message.encode('utf-8'))
            print(f"Message envoyé à l'Arduino : {message.strip()}")

            # Lire la réponse de l'Arduino (facultatif)
            if self.arduino.in_waiting > 0:
                response = self.arduino.readline().decode('utf-8').strip()
                print(f"Réponse de l'Arduino : {response}")

        except Exception as e:
            print(f"Erreur lors de l'envoi des données à l'Arduino : {e}")

    
        
    # Thread pour capturer les frames et leurs données
    def capture_frames_and_data(self):
        while True:
            if not self.frame_queue.full():
                # Capture d'une image
                frame = self.camera.picam2.capture_array()
                
                # Détection des objets
                detections = self.camera.get_detections()
                labels = self.camera.get_labels()

                # Rassembler les données de détection
                detection_data = {"detections": []}
                list_persons = []

                for detection in detections:
                    if int(detection.category) > len(labels) - 1:
                        continue
                    
                    label = labels[int(detection.category)]
                    if label != "person":  # Filtre uniquement pour les personnes
                        continue
                    
                    x, y, w, h = detection.box
                    cropped_person = frame[y:y+h, x:x+w]
                    
                    # Encodage du recadré pour éviter la perte d'information
                    _, buffer = cv2.imencode('.jpg', cropped_person)
                    list_persons.append(buffer.tobytes())
                    
                    detection_data['detections'].append({
                        "label": label,
                        "bbox": [x, y, w, h]
                    })

                # Mettre les images recadrées et les données dans la file
                self.frame_queue.put((list_persons, detection_data))
            
            time.sleep(DELAY)
    
    def run(self):
        """Lance le serveur Flask."""
        self.app.run(debug=True, host='0.0.0.0', port=5000)

    