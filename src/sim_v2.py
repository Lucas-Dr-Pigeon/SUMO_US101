# -*- coding: utf-8 -*-
import os, sys
import traci
import tqdm
import traci.constants as tc
import copy
import math


N_STEPS = 50
WORK_PATH = os.path.abspath(os.path.join(os.getcwd(), ".."))
SUMO_BINARY = "sumo.exe"
SUMO_CMD = [SUMO_BINARY, "-c", os.path.join(WORK_PATH, "data/us101.sumocfg")]
print("Starting sumo service ... ")
traci.start(SUMO_CMD)
TRACI_START = 1


class Simulation:
    def __init__(self):
        self.vehicleList = []
        self.step = 0
        
    def Check_Vehicles(self):
        for _veh in traci.vehicle.getIDList():
            if _veh not in self.vehicleList:
                self.vehicleList.append(_veh)
                
    def Run(self):
        for _step in tqdm.trange(N_STEPS):
            traci.simulationStep()
            time = traci.simulation.getCurrentTime()
            self.Check_Vehicles()
            
    def Capture(self, zone):
        for vehID in traci.vehicle.getIDList():
            _lane = traci.vehicle.getLaneID(vehID)
            _lanePos = traci.vehicle.getLanePosition(vehID)
            
        
            
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
        
    _index = 0
    
    if target_length <= thisLen - start_lane_pos:
        _dictLaneDist[start_lane] = [start_lane_pos, start_lane_pos + target_length, _index]
        _dictOfDict[start_lane] = _dictLaneDist
    else:
        _connLanes = traci.lane.getLinks(start_lane)
        _dictLaneDist[start_lane] = [start_lane_pos, thisLen]
        _sumLen += thisLen - start_lane_pos
        _ = GET_NEXT_CONNECTED_LANE(_connLanes, _dictLaneDist, _sumLen, _dictOfDict, target_length)
    return _dictOfDict
    
def GET_NEXT_CONNECTED_LANE(connLanes, dictLaneDist, sumLen, dictOfDict, target_length):
    for _next in connLanes:
        _nextLane = _next[0]
        # print (_nextLane)
        thisLen = traci.lane.getLength(_nextLane)
        if NOT_A_SEGMENT(_nextLane):
            continue
        if target_length <= thisLen + sumLen:
            # print (_nextLane)
            # print (target_length - sumLen)
            copy_dictLaneDist = copy.deepcopy(dictLaneDist)
            copy_dictLaneDist[_nextLane] = [0, target_length - sumLen]
            dictOfDict[_nextLane] = copy_dictLaneDist
        
        else:
            copy_dictLaneDist = copy.deepcopy(dictLaneDist)
            copy_dictLaneDist[_nextLane] = [0, thisLen]
            _sumLen = sumLen + thisLen
            _connLanes = traci.lane.getLinks(_nextLane)
            # print (_connLanes)
            GET_NEXT_CONNECTED_LANE(_connLanes, copy_dictLaneDist, _sumLen, dictOfDict, target_length)
            # print (copy_dictLaneDist)
            
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
        
        
    
    
            
        
        
        
        
    
    



if __name__ == "__main__":
    
    
    # sim = Simulation()
    # sim.Run()
    # vehicleList = sim.vehicleList
        
    # checkLanes = traci.lane.getLinks('122900739_0')
    # edge = traci.lane.getEdgeID('172076623_0')
    # laneNo = traci.edge.getLaneNumber('13277214#1')
    
    segmentDict = Get_A_SEGMENT('24813112_1', target_length=1000)
    
    # testLength = math.sqrt(
    #     (5146.64 - 5103.05)**2 + (6418.50 - 6508.44)**2
    #     )
    
    # testShape = traci.lane.getShape('859705684_1')
    # testLength = LANE_LENGTH_BY_NODES('859705684_1')
    # laneLength = traci.lane.getLength('859705684_1')
        
    # traci.close()
    
    

