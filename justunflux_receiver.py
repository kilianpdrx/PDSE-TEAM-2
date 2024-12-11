import requests
import cv2
import numpy as np

url = "http://10.11.6.148:5000/fusion"

# Envoyer une requête pour obtenir le flux vidéo
response = requests.get(url, stream=True)

if response.status_code == 200:
    print("Connexion établie avec le serveur.")

    byte_buffer = b''
    boundary = b'--frame'

    try:
        for chunk in response.iter_content(chunk_size=1024):
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
                image_id = None
                extra_value = None
                for line in headers.split('\r\n'):
                    if line.startswith('X-Image-ID:'):
                        image_id = line.split(':', 1)[1].strip()
                    elif line.startswith('X-Extra-Value:'):
                        extra_value = line.split(':', 1)[1].strip()

                # Décoder l'image
                image_array = np.frombuffer(image_data, dtype=np.uint8)
                frame = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

                if frame is not None:
                    # Afficher l'image et les métadonnées
                    print(f"Image ID: {image_id}, Extra Value: {extra_value}")
                    cv2.imshow('Frame', frame)

                    # Arrêter la boucle si 'q' est pressé
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
    except Exception as e:
        print(f"Erreur lors du traitement : {e}")
    finally:
        cv2.destroyAllWindows()
else:
    print(f"Erreur : impossible de se connecter au serveur. Code {response.status_code}")
