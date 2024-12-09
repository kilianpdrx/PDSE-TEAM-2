from flask import Flask, Response
import time
from ai_camera import IMX500Detector
import cv2


# Initialisation du modèle et de la caméra
model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
camera = IMX500Detector(model)

# Configuration Flask
app = Flask(__name__)

# Démarrage de la caméra avec aperçu activé
camera.start(show_preview=True)

@app.route('/video_feed')
def video_feed():
    def generate_frames():
        try:
            while True:
                frame_start_time = time.time()
                
                # Récupération des détections
                detections = camera.get_detections()
                labels = camera.get_labels()
                
                for detection in detections:
                    if int(detection.category) > len(labels) - 1:
                        continue
                    label = labels[int(detection.category)]
                    confidence = detection.conf
                    # if label == "person" and confidence > 0.4:
                    #     print(f"Person detected with {confidence:.2f} confidence!")

                # # Capture de la frame pour la diffusion
                frame = camera.picam2.capture_array()  # Capture la frame depuis le flux de la caméra
                _, buffer = cv2.imencode('.jpg', frame)  # Compression en format JPEG

                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

                frame_processing_time = time.time() - frame_start_time
                print(f"FPS: {1 / frame_processing_time:.2f}")
                time.sleep(0.001)  # Pause pour éviter la surcharge

        except GeneratorExit:
            print("Client déconnecté.")
        finally:
            camera.stop()
            print("Arrêt de la caméra.")
            exit()

    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
