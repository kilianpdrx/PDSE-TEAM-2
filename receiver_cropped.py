import cv2
import requests
import numpy as np
import json


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






# URL du flux vidéo, on peut mettre ça dans un navigateur pour voir le flux
url = {"cropped": "http://10.11.6.148:5000/cropped_feed",
       "full": "http://10.11.6.148:5000/full_feed",
       "data": "http://10.11.6.148:5000/data"}

CHOIX_URL = "cropped" # or full


# Connexion au flux vidéo
stream = requests.get(url[CHOIX_URL], stream=True)
if stream.status_code != 200:
    print("Impossible de se connecter au flux vidéo.")
    exit()

# Variables pour extraire les frames
bytes_data = b""
frame_counter = 0


if CHOIX_URL == "cropped" or CHOIX_URL == "full":
    for chunk in stream.iter_content(chunk_size=1024):
        bytes_data += chunk

        # Détecter les délimitations JPEG
        a = bytes_data.find(b'\xff\xd8')  # Début de l'image
        b = bytes_data.find(b'\xff\xd9')  # Fin de l'image

        if a != -1 and b != -1:
            # Extraire l'image JPEG
            jpeg_data = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]

            # Convertir en image OpenCV
            nparr = np.frombuffer(jpeg_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            if CHOIX_URL == "cropped":
                if not calibrated:
                    target_features = calibrate(extractor, frame)
                    if target_features is not None:
                        list_target_features.append(target_features)
                        print(f"Calibration : {len(list_target_features) / min_number_features * 100:.2f}%")

                    if len(list_target_features) >= min_number_features:
                        print("Calibration terminée.")
                        calibrated = True

                cv2.imshow("Cropped Person", frame)
            else:
                cv2.imshow("Full Frame", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

elif CHOIX_URL == "data":
    for line in stream.iter_lines():
        if line:
            try:
                # Charger les données JSON
                data = json.loads(line.decode('utf-8').replace("data: ", ""))

                # Afficher les données pour validation
                print(json.dumps(data, indent=4))

            except json.JSONDecodeError as e:
                print(f"Erreur de décodage JSON : {e}")
                continue

cv2.destroyAllWindows()