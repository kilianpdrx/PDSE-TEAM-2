from ai_camera import IMX500Detector
from serveur_utils import *
import time


from ai_camera import IMX500Detector






if __name__ == "__main__":
    model = "/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk"
    camera = IMX500Detector(model)
    camera.start(show_preview=False)
    time.sleep(2)


    serveur = Serveur(camera)
    threading.Thread(target=serveur.capture_frames_and_data, daemon=True).start()

    serveur.run()