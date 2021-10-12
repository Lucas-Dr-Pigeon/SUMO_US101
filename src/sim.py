# -*- coding: utf-8 -*-
import os, sys
import traci

N_STEPS = 3600
SUMO_BINARY = os.path.abspath(os.path.join(os.getcwd(), ".."))
SUMO_CMD = [SUMO_BINARY, "-c", "us101.sumocfg"]

if __name__ == "__main__":

    print("Starting sumo service ... ")

    traci.start(['C:/Users/dachu/PycharmProjects/stuttgart-sumo-traffic-scenario-master/simulation',
                 '-c', 'Simulation_sunday.sumo.cfg.xml'
                 ])

