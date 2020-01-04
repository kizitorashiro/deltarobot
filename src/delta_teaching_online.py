from datetime import datetime

from delta_teaching_offline import DeltaTeachingOffline
from ps4_controller import PS4Controller

class DeltaTeachingOnline(DeltaTeachingOffline):
    def __init__(self, config):
        super().__init__(config)
        
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
    "controller_type": "ps4",
    "view": "none",
    "load_limit": -1,
    "record_type": "motor_angle"
}


delta = DeltaTeachingOnline(config)
delta.start()
delta.end()
filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".online.json"
delta.save_history(filename)


