from ai_camera import IMX500Detector
from serveur_utils import *





if __name__ == "__main__":
    serveur = Serveur()
    threading.Thread(target=serveur.capture_frames_and_data, daemon=True).start()

    serveur.run()