
import numpy as np
from datetime import datetime
import sys

from delta_base import DeltaBase
from ax12_driver import AX12Driver
from file_controller import FileController

class DeltaPlayerOffline(DeltaBase):

    def __init__(self, config, filename):
        super().__init__(config)
        self.filename = filename

    def init_controller(self):
        self.controller = FileController(self.model, self.config, self.filename)

    def actuate(self):
        return True, None, None


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
    "tty": None, 
    "view": "pyplot",
    "load_limit": -1,
    "record_type": ""
}


if __name__ == '__main__':
    delta = DeltaPlayerOffline(config, sys.argv[1])
    delta.start()
    delta.end()

