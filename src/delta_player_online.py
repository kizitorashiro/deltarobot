from datetime import datetime
import sys
from delta_player_offline import DeltaPlayerOffline
import time
class DeltaPlayerOnline(DeltaPlayerOffline):
    def __init__(self, config, filename):
        super().__init__(config, filename)
        
    def actuate(self):
        
        thetas = self.model.get_thetas()
        return self.driver.move_arms(thetas)
        
        

config = {
    "arm_length": {
        "A":  55.0, #mm
        "B":  87.0, #mm
        "C": 171.0, #mm
        "D":  51.0  #mm
    },
    "move_min": [-60.0, -60.0, -200.0],     #[x, y, z]
    "move_max": [ 60.0,  60.0, -110.0],     #[x, y, z]
    "maxspeed": 100.0, #mm/sec 80
    "plot_range": {
        "x": [-200.0, 200.0], #[min, max]
        "y": [-200.0, 200.0], #[min, max]
        "z": [-300.0, 100.0]  #[min, max]
    },
    # "tty": "/dev/tty.usbserial-A5052NCY"
    "tty": "COM5",
    "view": "none",
    "load_limit": 512,
    "record_type": ""
}

if __name__ == '__main__':
    delta = DeltaPlayerOnline(config, sys.argv[1])
    delta.start()
    delta.end()


