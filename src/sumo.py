# -*- coding: utf-8 -*-
import os, sys
import traci

BEGIN_VALUE = 0
END_VALUE = 3600
N_STEPS = 250
STEP_LENGTH = 2
WORK_PATH = os.path.abspath(os.path.join(os.getcwd(), ".."))
SUMO_BINARY = "sumo-gui.exe"
SUMO_CMD = [ SUMO_BINARY, "-c", os.path.join(WORK_PATH, "data/us101.sumocfg"),
             "--step-length", str(STEP_LENGTH)
             # "--begin value", str(BEGIN_VALUE)
             # "--end value", str(END_VALUE)
             ]
print("Starting sumo service ... ")
traci.start(SUMO_CMD)
TRACI_START = 1

class Sumo:
    def __init__(self):
        self.begin = BEGIN_VALUE
        self.end = END_VALUE
        self.step_length = STEP_LENGTH
        self.n_steps = N_STEPS