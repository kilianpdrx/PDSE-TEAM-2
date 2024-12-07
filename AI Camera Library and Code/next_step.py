from ai_camera import IMX500Detector
import time
import cv2
from functions_final import *

camera = IMX500Detector("/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk")

# Start the detector with preview window
camera.start(show_preview=True)


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



try:
    # Boucle principale
    while True:
        frame_start_time = time.time()

        # Récupérer les détections
        detections = camera.get_detections()
        labels = camera.get_labels()

        for detection in detections:
            label = labels[int(detection.category)]
            confidence = detection.conf
            # if label == "person" and confidence > 0.4:
            #     print(f"Person detected with {confidence:.2f} confidence!")






        # frame = camera.picam2.capture_array()
        # camera.maxi_test(frame)
        # cv2.imshow("Frame", frame)
        # cv2.waitKey(1)
        
        




        frame_processing_time = time.time() - frame_start_time
        print(f"FPS: {1 / frame_processing_time:.2f}")
        total_processing_time += frame_processing_time
        frame_count += 1

        # Petite pause pour éviter de surcharger le système
        time.sleep(0.001)




except KeyboardInterrupt:
    # Arrêter le programme proprement avec Ctrl+C
    print("\nInterruption reçue, arrêt du programme...")




finally:
    # Arrêter la caméra et libérer les ressources
    camera.stop()
    end_time = time.time()
    total_time = end_time - start_time
    average_processing_time = total_processing_time / frame_count if frame_count > 0 else -1

    print(" ")
    print(f"Temps total de traitement de la vidéo : {total_time:.2f} secondes")
    print(f"Temps moyen de traitement par image : {average_processing_time:.4f} secondes")
    print(f"FPS moyen : {1 / average_processing_time:.2f} FPS")