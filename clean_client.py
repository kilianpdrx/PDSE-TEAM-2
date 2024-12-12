
from client_utils import *

CHOIX_FLUX = [3] # 0: full, 1: cropped, 2: data 3: fusion

client = Client()


if __name__ == "__main__":
    
    for t in CHOIX_FLUX:
        client.threads[t].start()

    # Thread principal pour afficher les flux et les données
    client.display_streams2()
