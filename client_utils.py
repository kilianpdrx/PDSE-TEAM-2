import cv2
import time
import requests
import numpy as np
import threading
import queue
import json
import torch


from FE_modif import FeatureExtractor2



IDED_PERSON = 1
BAD_PERSON = -1

target_features = None  # Caractéristiques de la personne calibrée
list_target_features = []  # Liste des caractéristiques de la personne calibrée
min_number_features = 30  # Nombre minimal de features pour la calibration
calibrated = False  # Statut de calibration
DELAY = 0.05
SIM_THRESHOLD = 0.7


index_frame = 0


def calibrate(extractor, cropped_person):
    global index_frame
    # we skip some frames to let the person turn on himself
    index_frame += 1
    
    if index_frame % 30 == 0:
        return None
    # Détecter les caractéristiques de la personne
    target_features = extractor(cropped_person)
    return target_features


def compare2(frame, extractor, list_target_features):
    """Comparer les caractéristiques des personnes détectées avec celles de la personne calibrée."""
    
    features = extractor(frame)
    mean = 0

    
    # si la moyenne des similarités est supérieure au seuil, on considère que la personne est ciblée
    for target_features in list_target_features:
        if target_features is not None:
            # Calcul de la similarité entre les caractéristiques extraites et celles de la cible
            similarity = torch.nn.functional.cosine_similarity(features, target_features).item()
            mean += similarity
    
    total_val = mean/len(list_target_features)

    if total_val > SIM_THRESHOLD:
        print("Personne ciblée")
        track_id = IDED_PERSON
    else:
        print("Personne non ciblée")
        track_id = BAD_PERSON

    return track_id





class Client:
    def __init__(self, show_person=False, wait_for_input=True, IP_address=""):

        self.IP = IP_address
        
        
        self.urls = {
            "cropped": f"http://{self.IP}:5000/cropped_feed",
            "full": f"http://{self.IP}:5000/full_feed",
            "data": f"http://{self.IP}:5000/data",
            "fusion": f"http://{self.IP}:5000/fusion"
        }
        self.POST_URL = f"http://{self.IP}:5000/update_data"
        
    
        self.full_frame_queue = queue.Queue(maxsize=1)
        self.cropped_frame_queue = queue.Queue(maxsize=1)
        self.data_queue = queue.Queue(maxsize=1)
        self.final_queue = queue.Queue(maxsize=1000)

        
        self.list_target_features = []
        
        self.show_cropped = show_person
        self.wait_for_input = wait_for_input
        
        
        self.extractor = FeatureExtractor2(
            model_name='osnet_x0_25',
            model_path='osnet_x0_25_imagenet.pth',
            device='cpu'
        )
    

        self.threads = [
            threading.Thread(target=self.fetch_full_feed, daemon=True),
            threading.Thread(target=self.fetch_cropped_feed, daemon=True),
            threading.Thread(target=self.fetch_data_feed, daemon=True),
            threading.Thread(target=self.fetch_final_flux, daemon=True),
            threading.Thread(target=self.fetch_final_flux_auto, daemon=True)
        ]
        
        self.latest_frame = None
        self.lock = threading.Lock()
    
    
    
    
    def send_data_to_server(self,data):
        try:
            response = requests.post(self.POST_URL, json=data)
            if response.status_code == 200:
                # print("Données envoyées avec succès.")
                pass
            else:
                print(f"Erreur lors de l'envoi : {response.status_code}, {response.text}")
        except Exception as e:
            print(f"Erreur : {e}")
    
    
    def fetch_full_feed(self):
        """Thread pour le flux vidéo complet."""
        stream = requests.get(self.urls["full"], stream=True)
        if stream.status_code != 200:
            print("Impossible de se connecter au flux full_feed.")
            return

        bytes_data = b""
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')

            if a != -1 and b != -1:
                jpeg_data = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]

                # Convertir en numpy array pour OpenCV
                nparr = np.frombuffer(jpeg_data, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                self.full_frame_queue.put(frame)  # Ajouter à la queue


    def fetch_cropped_feed(self):
        """Thread pour le flux des images recadrées."""
        stream = requests.get(self.urls["cropped"], stream=True)
        if stream.status_code != 200:
            print("Impossible de se connecter au flux cropped_feed.")
            return

        bytes_data = b""
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_data += chunk
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')

            if a != -1 and b != -1:
                jpeg_data = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]

                # Convertir en numpy array pour OpenCV
                nparr = np.frombuffer(jpeg_data, np.uint8)
                cropped_person = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                self.cropped_frame_queue.put(cropped_person)  # Ajouter à la queue


    def fetch_data_feed(self):
        """Thread pour le flux JSON."""
        stream = requests.get(self.urls["data"], stream=True)
        if stream.status_code != 200:
            print("Impossible de se connecter au flux data.")
            return

        for line in stream.iter_lines():
            if line:
                try:
                    data = json.loads(line.decode('utf-8').replace("data: ", ""))
                    self.data_queue.put(data)  # Ajouter à la queue
                except json.JSONDecodeError:
                    print("Erreur lors de la lecture du flux JSON.")


    def fetch_final_flux(self):
        stream = requests.get(self.urls["fusion"], stream=True)
        if stream.status_code == 200:
            print("Connexion établie avec le serveur.")

            byte_buffer = b''
            boundary = b'--frame'
        
            try:
                print("Début de la lecture du flux fusion.")
                for chunk in stream.iter_content(chunk_size=1024):
                    byte_buffer += chunk

                    while boundary in byte_buffer:
                        # Trouver les délimitations des frames
                        frame_start = byte_buffer.find(boundary)
                        frame_end = byte_buffer.find(boundary, frame_start + len(boundary))

                        if frame_end == -1:
                            break

                        # Extraire une seule frame avec ses en-têtes
                        frame = byte_buffer[frame_start:frame_end]
                        byte_buffer = byte_buffer[frame_end:]

                        # Séparer les en-têtes du contenu de l'image
                        header_end = frame.find(b'\r\n\r\n')
                        headers = frame[:header_end].decode('utf-8')
                        image_data = frame[header_end + 4:]

                        # Lire les en-têtes personnalisés
                        height_box = None
                        dist_x = None
                        for line in headers.split('\r\n'):
                            if line.startswith('X-bbox:'):
                                height_box = line.split(':', 1)[1].strip()
                            elif line.startswith('X-dist_x:'):
                                dist_x = line.split(':', 1)[1].strip()

                        # Décoder l'image
                        image_array = np.frombuffer(image_data, dtype=np.uint8)
                        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        with self.lock:
                            self.latest_frame = (frame, dist_x, height_box)
                        # print("Updated the latest frame.")
                        
                        # self.final_queue.put((frame, dist_x, height_box))
                        # print("I read the frame and put it in the queue")
                        
            except Exception as e:
                print(f"Erreur lors du traitement : {e}")
            finally:
                cv2.destroyAllWindows()
        
        else:
            print(f"Erreur : impossible de se connecter au serveur. Code {stream.status_code}")

    def fetch_final_flux_auto(self):
        """Thread pour le flux fusion avec reconnexion automatique."""
        while True:
            try:
                stream = requests.get(self.urls["fusion"], stream=True, timeout=20)
                if stream.status_code != 200:
                    print("Impossible de se connecter au flux fusion.")
                    time.sleep(2)  # Attendre avant de réessayer
                    continue

                byte_buffer = b''
                boundary = b'--frame'
                print("Début de la lecture du flux fusion.")
                for chunk in stream.iter_content(chunk_size=1024):
                    byte_buffer += chunk

                    while boundary in byte_buffer:
                        # Trouver les délimitations des frames
                        frame_start = byte_buffer.find(boundary)
                        frame_end = byte_buffer.find(boundary, frame_start + len(boundary))

                        if frame_end == -1:
                            break

                        # Extraire une seule frame avec ses en-têtes
                        frame = byte_buffer[frame_start:frame_end]
                        byte_buffer = byte_buffer[frame_end:]

                        # Séparer les en-têtes du contenu de l'image
                        header_end = frame.find(b'\r\n\r\n')
                        headers = frame[:header_end].decode('utf-8')
                        image_data = frame[header_end + 4:]

                        # Lire les en-têtes personnalisés
                        height_box = None
                        dist_x = None
                        for line in headers.split('\r\n'):
                            if line.startswith('X-bbox:'):
                                height_box = line.split(':', 1)[1].strip()
                            elif line.startswith('X-dist_x:'):
                                dist_x = line.split(':', 1)[1].strip()

                        # Décoder l'image
                        image_array = np.frombuffer(image_data, dtype=np.uint8)
                        frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
                        with self.lock:
                            self.latest_frame = (frame, dist_x, height_box)
                        # print("Updated the latest frame.")
                        
                        # self.final_queue.put((frame, dist_x, height_box))
                        # print("I read the frame and put it in the queue")
            except requests.RequestException as e:
                print(f"Erreur de connexion au flux fusion : {e}")
                time.sleep(2)  # Attendre avant de réessayer
            except Exception as e:
                print(f"Erreur inattendue : {e}")
                time.sleep(2)  # Attendre avant de réessayer

    def display_streams2(self):
        global calibrated
        while True:
            cropped_frame = None
            
            # if not self.final_queue.empty():
            #     cropped_frame, x_dist, height_box = self.final_queue.get()
            
            with self.lock:
                latest_frame = self.latest_frame
            
            if latest_frame is not None:
                # Décomposer les valeurs si elles existent
                cropped_frame, x_dist, height_box = latest_frame


            
                # Calibration
                if cropped_frame is not None:
                    
                    # no one is on the frame but we still display it
                    if x_dist == -1 or height_box == -1:
                        continue
                    else:
                        if not calibrated:
                            target_features = calibrate(self.extractor, cropped_frame)
                            if target_features is not None:
                                self.list_target_features.append(target_features)
                                print(f"Calibration : {len(self.list_target_features) / min_number_features * 100:.2f}%")
                            else: continue # skip the rest of the loop
                            
                            if len(self.list_target_features) >= min_number_features:
                                print("Calibration terminée.")
                                calibrated = True
                                
                                if self.wait_for_input:
                                    input("Press Enter to continue...")
                            
                            # time.sleep(0.2) # otherwise the calibration would be too fast
                        else:
                            tracking = compare2(cropped_frame, self.extractor, self.list_target_features)
                            data_to_send = {
                                "x_distance": x_dist,
                                "height_box": height_box, 
                                "tracking": tracking
                            }
                            self.send_data_to_server(data_to_send)
                            # print(f"Distance en x : {x_dist}")
                
                if self.show_cropped:
                    cropped_frame = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2RGB)
                    cv2.imshow("Cropped Person", cropped_frame)
                        

            time.sleep(DELAY)
            # Fermer les fenêtres si 'q' est pressé
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()








