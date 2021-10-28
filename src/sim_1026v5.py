# -*- coding: utf-8 -*-
import os, sys
import traci
import tqdm
import traci.constants as tc
import copy
import math
import matplotlib.pyplot as plt
import numpy as np
from DetectionZone1026v1 import detectionZone, detectorSet
import sumo
import graph

class Simulation(sumo.Sumo):
    def init_detectors(self, detectorParams):
        self.detectors = None
        if detectorParams:
            self.detectors = detectorSet(detectorParams)
    
    def __init__(self, detectorParams=None):
        super().__init__()
        self.step = 0
        self.vehicleSub = set()
        self.SubscriptionResults = []
        self.init_detectors(detectorParams)

                
    def Run(self):   
        for _step in tqdm.trange(self.n_steps):
            traci.simulationStep()
            if self.detectors:
                self.vehicleSub = self.detectors.scan(self.vehicleSub)
                self.detectors.detectors_collect()
            self.step += 1
        # self.detected_trespassing = self.detector.get_all_trespassing(self.SubscriptionResults)
        

if __name__ == "__main__":
    ''' detector configurations '''
    detectorParameters = [
        [['25003401-AddedOnRampEdge_1', 2000, 0], "sectioned", 10]
        # , [['25003401_2', 2000, 100], "sectioned", 10]
        # , [['866618593_2', 2000, 100], "sectioned", 10]
        ]
    ''' run simulation '''
    sim = Simulation(detectorParams = detectorParameters)
    sim.Run()
    ''' collect traffic states from zone detectors '''
    allstates = [sim.detectors.detectors[_i].allstates for _i in range(len(sim.detectors.detectors))]
    ''' report the simulation time '''
    print (("Simulation complete at: %s s")%(traci.simulation.getTime()))
    ''' plot a q-k diagram '''
    for i, state in enumerate(allstates):
        graph.plot_states(state, detectorParameters[i][0])
        
    tresArray_step02 = sim.detectors.detectors[0].tresArray
    tresDict_step02 = sim.detectors.detectors[0].tresDict
    detected_vehicles02 = sim.detectors.detectors[0].detectedVehicles
    subs_02 = sim.detectors.detectors[0].subscriptionResults
    detected_02 = len(sim.detectors.detectors[0].detectedVehicles)
    
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
    
    

