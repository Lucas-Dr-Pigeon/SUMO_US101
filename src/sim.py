# -*- coding: utf-8 -*-
import os, sys
import traci
import tqdm
import traci.constants as tc
import copy
import math


N_STEPS = 2000
STEP_LENGTH = 0.5
WORK_PATH = os.path.abspath(os.path.join(os.getcwd(), ".."))
SUMO_BINARY = "sumo.exe"
SUMO_CMD = [ SUMO_BINARY, "-c", os.path.join(WORK_PATH, "data/us101.sumocfg"),
             "--step-length", str(STEP_LENGTH)             ]
print("Starting sumo service ... ")
traci.start(SUMO_CMD)
TRACI_START = 1


class Simulation:
    def __init__(self):
        self.vehicleList = []
        self.step = 0
        self.vehicleSub = []
        self.SubscriptionResults = []
        
    def Check_Vehicles(self):
        for _veh in traci.vehicle.getIDList():
            if _veh not in self.vehicleList:
                self.vehicleList.append(_veh)
                
    def Run(self, capture = None):           
        for _step in tqdm.trange(N_STEPS):
            traci.simulationStep()
            if capture:
                self.Capture(capture)
            time = traci.simulation.getCurrentTime()
            self.SubscriptionResults.append(traci.vehicle.getAllSubscriptionResults())
            self.step += 1
            
            
    def Capture(self, detectionZone):
        for laneID in detectionZone:
            vehDetectList = traci.lane.getLastStepVehicleIDs(laneID)
            for vehID in self.vehicleSub:
                if traci.vehicle.getLaneID(vehID) not in detectionZone:
                    self.vehicleSub.remove(vehID)
                    traci.vehicle.unsubscribe(vehID)
            for vehID in vehDetectList:
                if vehID not in self.vehicleSub:
                    self.vehicleSub.append(vehID)
                    traci.vehicle.subscribe(vehID, (tc.VAR_LANEPOSITION, tc.VAR_DISTANCE, tc.VAR_LANE_ID))

def Get_A_SEGMENT(start_lane: str, target_length=1000, start_lane_pos=100)->dict:
    if not TRACI_START:
        raise ("Start traci first!")
    _sumLen = 0
    _dictLaneDist = {}
    _dictOfDict = {}
    thisLen = traci.lane.getLength(start_lane)
    
    start_lane_pos = min(start_lane_pos, thisLen)
    
    if NOT_A_SEGMENT(start_lane):
        raise ("The starting lane was on a ramp!")
    
    ''' initiate the order of lanes to 0 '''
    _index = 0
    
    ''' first lane is long enough? first lane isn't long enough? ''' 
    if target_length <= thisLen - start_lane_pos:
        _dictLaneDist[start_lane] = [start_lane_pos, start_lane_pos + target_length, _index]
        _dictOfDict[start_lane] = _dictLaneDist
    else:
        _connLanes = traci.lane.getLinks(start_lane)
        _dictLaneDist[start_lane] = [start_lane_pos, thisLen, _index]
        _sumLen += thisLen - start_lane_pos
        _ = GET_NEXT_CONNECTED_LANE(_connLanes, _dictLaneDist, _sumLen, _dictOfDict, target_length, _index)
    return _dictOfDict
    
def GET_NEXT_CONNECTED_LANE(connLanes, dictLaneDist, sumLen, dictOfDict, target_length, index):
    for _next in connLanes:
        _nextLane = _next[0]
        thisLen = traci.lane.getLength(_nextLane)
        if NOT_A_SEGMENT(_nextLane):
            continue
        _index = index
        _index += 1
        if target_length <= thisLen + sumLen:
            copy_dictLaneDist = copy.deepcopy(dictLaneDist)
            copy_dictLaneDist[_nextLane] = [0, target_length - sumLen, _index]
            dictOfDict[_nextLane] = copy_dictLaneDist
        
        else:
            copy_dictLaneDist = copy.deepcopy(dictLaneDist)
            copy_dictLaneDist[_nextLane] = [0, thisLen, _index]
            _sumLen = sumLen + thisLen
            _connLanes = traci.lane.getLinks(_nextLane)
            GET_NEXT_CONNECTED_LANE(_connLanes, copy_dictLaneDist, _sumLen, dictOfDict, target_length, _index)

            
def NOT_A_SEGMENT(thisLane):
    edge = traci.lane.getEdgeID(thisLane)
    LaneNo = traci.edge.getLaneNumber(edge)
    if LaneNo < 2:
        return True

def LANE_LENGTH_BY_NODES(thisLane):
    shape = traci.lane.getShape(thisLane)
    _len = 0
    for i in range(len(shape)-1):
        _from = shape[i]
        _to = shape[i+1]
        _len += math.sqrt(
            (_from[0] - _to[0])**2 + (_from[1] - _to[1])**2
            )
    return _len
        
def CONCATE_TRAVELED_DISTANCE(subList):
    0
    
    
            
        
        
        
        
    
    



if __name__ == "__main__":
    segmentDict = Get_A_SEGMENT('25003401-AddedOnRampEdge_1', 2000, 100)
    # segmentDict = Get_A_SEGMENT('27287058_0', target_length=1000)
    detectionZone = list(segmentDict[list(segmentDict.keys())[0]].keys())
    
    sim = Simulation()
    sim.Run(capture=detectionZone)
    
    lastSubResults = sim.SubscriptionResults
    
        
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
    
    

