# -*- coding: utf-8 -*-
import os, sys
import traci
import tqdm
import traci.constants as tc
import copy
import math
import matplotlib.pyplot as plt
import numpy as np
from DetectionZone import detectionZone
import sumo
import graph

class Simulation(sumo.Sumo):
    def __init__(self, detector=None):
        super().__init__()
        self.vehicleList = []
        self.step = 0
        self.vehicleSub = []
        self.SubscriptionResults = []
        self.detector = None
        if detector:
            self.detector = detectionZone(*detector)        
        
    def Check_Vehicles(self):
        for _veh in traci.vehicle.getIDList():
            if _veh not in self.vehicleList:
                self.vehicleList.append(_veh)
                
    def Run(self):   
        for _step in tqdm.trange(self.n_steps):
            traci.simulationStep()
            if self.detector:
                self.detector.scan(self.vehicleSub, self.SubscriptionResults)
            
            self.step += 1
        self.detected_trespassing = self.detector.get_all_trespassing(self.SubscriptionResults)


if __name__ == "__main__":
    
    detectorParameters = [['24794598#0.51_1', 2000, 100], "sectioned", 10]
    sim = Simulation(detector = detectorParameters)
    sim.Run()
    allstates = sim.detector.allstates
    print ("sim time: ", traci.simulation.getTime())
    graph.plot_states(allstates)
    
    # segmentDict = Get_A_SEGMENT('25003401-AddedOnRampEdge_3', target_length=2000, start_lane_pos=0)
    # segmentDict = Get_A_SEGMENT('172076623_0', target_length=1000)

    # detectionZone = [zone[2] for zone in list(segmentDict.values())[0]]
    
    # sim = Simulation()
    # sim.Run(capture=detectionZone)
    
    # lastSubResults = sim.SubscriptionResults
    
    # detectionZoneList = list(segmentDict.values())[0]
    # detectionZoneDict = SEGMENT_LIST_TO_DICT(detectionZoneList)
    
    # tripsDict = CONCATE_VEHICLE_SUB(lastSubResults, list(segmentDict.values())[0])
    
    # Plot_Dist_Error_Distribution(tripsDict)
        
    # checkLanes = traci.lane.getLinks('122900739_0')
    # edge = traci.lane.getEdgeID('172076623_0')
    # laneNo = traci.edge.getLaneNumber('13277214#1')
    
    
    
    # testLength = math.sqrt(
    #     (5146.64 - 5103.05)**2 + (6418.50 - 6508.44)**2
    #     )
    
    # testShape = traci.lane.getShape('859705684_1')
    # testLength = LANE_LENGTH_BY_NODES('859705684_1')
    # laneLength = traci.lane.getLength('859705684_1')
        
    # traci.close()
    
    

