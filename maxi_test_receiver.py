import requests
import cv2
import numpy as np
import json
import time

from FE_modif import FeatureExtractor2
from functions_final import *



def calibrate(extractor, cropped_person):
    # Détecter les caractéristiques de la personne
    target_features = extractor(cropped_person)


    return target_features


extractor = FeatureExtractor2(
    model_name='osnet_x0_25',
    model_path='osnet_x0_25_imagenet.pth',
    device='cpu'
)

def process_combined_feed():
    global calibrated, frame_count, target_features, list_target_features, index_frame, min_number_features, similarity_threshold, start_time
    
    url = 'http://10.11.6.148:5000/combined_feed'  # Remplacez par l'URL correcte du flux combiné
    with requests.get(url, stream=True) as response:
        if response.status_code != 200:
            print(f"Erreur de connexion au serveur : {response.status_code}")
            return

        bytes_data = b""
        while True:
            chunk = response.raw.read(1024)
            bytes_data += chunk

            if b'--frame' in bytes_data:
                # Chercher les délimitations entre l'image et les données
                frame_start = bytes_data.find(b'\xff\xd8')  # Début de l'image JPEG
                frame_end = bytes_data.find(b'\xff\xd9')    # Fin de l'image JPEG
                json_start = bytes_data.find(b'{"timestamp"')  # Début des données JSON
                json_end = bytes_data.find(b'\r\n\r\n', json_start)  # Fin des données JSON

                if frame_start != -1 and frame_end != -1:
                    jpeg_data = bytes_data[frame_start:frame_end + 2]
                    bytes_data = bytes_data[frame_end + 2:]

                    # Convertir en format OpenCV
                    nparr = np.frombuffer(jpeg_data, np.uint8)
                    if nparr.size == 0:
                        print("Erreur : les données d'image sont vides")
                        continue

                    frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    
                    if not calibrated:
                        # Étape de calibration
                        target_features = calibrate(extractor, frame)
                        if target_features is not None:
                            print(f"Calibration : {index_frame/min_number_features * 100:.2f}%")
                            list_target_features.append(target_features)
                            
                        else:
                            print("Veuillez vous placer devant la caméra pour la calibration.")
                                
                        if len(list_target_features) >= min_number_features:
                            print("Calibration terminée. Personne cible enregistrée.")
                            calibrated = True
                            print("Durée de la calibration : ", time.time() - start_time)
                            # cv2.imshow("Personne calibrée", target_image)
                            # cv2.waitKey(0)
                            # cv2.destroyWindow("Personne calibrée")
                            
                    if frame is not None:
                        frame_count += 1
                        # Afficher l'image reçue
                        cv2.imshow('Image reçue', frame)
                        
                        
                    
                    
                    
                    

                # if json_start != -1 and json_end != -1:
                #     json_data = bytes_data[json_start:json_end].decode('utf-8')
                #     bytes_data = bytes_data[json_end + 4:]
                #     print(f"Données reçues: {json_data}")

                #     # Décoder les données JSON
                #     try:
                #         detection_data = json.loads(json_data)
                #         print(f"Données de détection reçues: {detection_data}")
                #     except json.JSONDecodeError:
                #         print("Erreur lors du décodage des données JSON")

                # Si la touche 'q' est pressée, arrêter le programme
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break



target_features = None  # Caractéristiques de la personne calibrée
list_target_features = []  # Liste des caractéristiques de la personne calibrée
list_target_features2 = []  # Liste des caractéristiques de la personne calibrée

min_number_features = 30  # Nombre minimal de features pour la calibration
calibrated = False  # Statut de calibration
similarity_threshold = 0.75  # Seuil de similarité pour la comparaison de ReID

index_frame = 0 # on est obligés d'en mettre 2 parce qu'on saute des frames
frame_count = 0
total_processing_time = 0.0
start_time = time.time()
frame_jump = 1  # Ignorer certaines frames pour optimiser











cv2.destroyAllWindows()
process_combined_feed()
