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


N_STEPS = 500
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
        self.vehicleSub = []
        self.SubscriptionResults = []
        
    def Check_Vehicles(self):
        for _veh in traci.vehicle.getIDList():
            if _veh not in self.vehicleList:
                self.vehicleList.append(_veh)
                
    def Run(self, detector = None):   
        if detector:
            self.detector = detectionZone(detector)        
        for _step in tqdm.trange(N_STEPS):
            traci.simulationStep()
            if detector:
                self.Capture()
            time = traci.simulation.getCurrentTime()
            self.SubscriptionResults.append(
                copy.deepcopy(traci.vehicle.getAllSubscriptionResults())
                )
            self.step += 1
        self.detected_trespassing = self.detector.get_all_trespassing(self.SubscriptionResults)
            
    def Capture(self):
        for laneID in self.detector.coveredEdges:
            vehDetectList = traci.lane.getLastStepVehicleIDs(laneID)
            for vehID in self.vehicleSub:
                if traci.vehicle.getLaneID(vehID) not in self.detector.coveredEdges:
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
    _LaneList = []
    _dictOfDict = {}
    thisLen = traci.lane.getLength(start_lane)
    
    start_lane_pos = min(start_lane_pos, thisLen)
    
    if NOT_A_SEGMENT(start_lane):
        raise ("The starting lane was on a ramp!")
    
    ''' initiate the order of lanes to 0 '''
    _index = 0
    
    ''' first lane is long enough? first lane isn't long enough? ''' 
    if target_length <= thisLen - start_lane_pos:
        _LaneList.append([start_lane_pos, start_lane_pos + target_length, start_lane])
        _dictOfDict[start_lane] = _LaneList
    else:
        _connLanes = traci.lane.getLinks(start_lane)
        _LaneList.append([start_lane_pos, thisLen, start_lane])
        _sumLen += thisLen - start_lane_pos
        _ = GET_NEXT_CONNECTED_LANE(_connLanes, _LaneList, _sumLen, _dictOfDict, target_length, _index)
    return _dictOfDict
    
def GET_NEXT_CONNECTED_LANE(connLanes, LaneList, sumLen, dictOfDict, target_length, index):
    for _next in connLanes:
        _nextLane = _next[0]
        thisLen = traci.lane.getLength(_nextLane)
        if NOT_A_SEGMENT(_nextLane):
            continue
        _index = index
        _LaneList = copy.deepcopy(LaneList)
        _index += 1
        if target_length <= thisLen + sumLen:
            _LaneList.append([0, target_length - sumLen, _nextLane])
            dictOfDict[_nextLane] = _LaneList
        
        else:
            _LaneList.append([0, thisLen, _nextLane])
            _sumLen = sumLen + thisLen
            _connLanes = traci.lane.getLinks(_nextLane)
            GET_NEXT_CONNECTED_LANE(_connLanes, _LaneList, _sumLen, dictOfDict, target_length, _index)

            
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

        
def CONCATE_VEHICLE_SUB(subList, detectionZoneList):
    _subList = copy.deepcopy(subList)
    null_trip = {"distance":0, "timespan":0, "accum_lane_pos":0, "trespass":0}
    tripsDict = {}
    for timestamp in range(0, len(_subList)):
        if not timestamp < len(subList)-1:
            continue
        for vehID in _subList[timestamp]:
            if _subList[timestamp][vehID] != "checked":
                if not IF_DETECTED(timestamp, vehID, subList, detectionZoneList):
                    continue
                tripsDict[(vehID, timestamp)] = copy.deepcopy(null_trip)
                UPDATE_CHECK(tripsDict[(vehID, timestamp)], _subList, timestamp, vehID, detectionZoneList)
                if tripsDict[(vehID, timestamp)] == null_trip:
                    del tripsDict[(vehID, timestamp)]
                _subList[timestamp][vehID] = "checked"
    return tripsDict    
    
def UPDATE_CHECK(tripInfo, subList, timestamp, vehID, detectionZoneList):
    if vehID in subList[timestamp+1]:
        if IF_DETECTED(timestamp+1, vehID, subList, detectionZoneList):
            tripInfo["distance"] += subList[timestamp+1][vehID][132] - subList[timestamp][vehID][132]
            tripInfo["timespan"] += 1
            tripInfo["accum_lane_pos"] += GET_ACCUMULATED_LANE_POSITION(detectionZoneList, laneID=subList[timestamp+1][vehID][81], lanePos=subList[timestamp+1][vehID][86]) \
                - GET_ACCUMULATED_LANE_POSITION(detectionZoneList, laneID=subList[timestamp][vehID][81], lanePos=subList[timestamp][vehID][86])
            ''' if trespass ? '''
            if subList[timestamp][vehID][81] != subList[timestamp+1][vehID][81]:
                tripInfo["trespass"] += 1
            ''' to prevent keyerror '''
            if timestamp+1 < len(subList)-1:
                UPDATE_CHECK(tripInfo, subList, timestamp+1, vehID, detectionZoneList)
        subList[timestamp+1][vehID]= "checked"

def IF_DETECTED(timestamp:int, vehID:str, subList, detectionZoneList)->bool:
    laneID = subList[timestamp][vehID][81]
    lanePos = subList[timestamp][vehID][86]
    detectionZoneDict = SEGMENT_LIST_TO_DICT(detectionZoneList)
    try:
        if detectionZoneDict[laneID]['start'] < lanePos <= detectionZoneDict[laneID]['end']:
            return True
        else:
            return False
    except KeyError:
        print (
            ("Unexpected lane ID: %s caused by vehicle %s at time %s!") %
            (laneID, vehID, timestamp)
            )
    if  laneID not in detectionZoneDict.keys():
        return False
    if not detectionZoneDict[laneID]['start'] < lanePos <= detectionZoneDict[laneID]['end']:
        return False
    else:
        return True                                     
            
def SEGMENT_LIST_TO_DICT(detectionZoneList):
    _dict = {}
    for i, segment in enumerate(detectionZoneList):
        _dict[segment[2]] = {"index": i, "start": segment[0], "end": segment[1]}
    return _dict
    
def GET_ACCUMULATED_LANE_POSITION(detectionZoneList, laneID, lanePos):
    detectionZoneDict = SEGMENT_LIST_TO_DICT(detectionZoneList)
    if detectionZoneDict[laneID]["index"] == 0:
        accumLanePos = lanePos - detectionZoneDict[laneID]["start"]
    else:
        _index = detectionZoneDict[laneID]["index"]
        accumLanePos = lanePos
        for i in range(_index):
            accumLanePos += detectionZoneList[i][1] - detectionZoneList[i][0]
    return accumLanePos

def Plot_Dist_Error_Distribution(tripsDict):
    
    accum_lane_pos = np.asarray(
            [tripsDict[_key]['accum_lane_pos'] for _key in tripsDict ]
        )
    dist = np.asarray(
            [tripsDict[_key]['distance'] for _key in tripsDict ]
        )
    
    timespan = np.asarray(
            [tripsDict[_key]['timespan'] for _key in tripsDict ]
        )
    
    plt.hist(
        accum_lane_pos - dist,
        density = True,
        color = "green", bins = 50
        )
    plt.show()
    plt.scatter(timespan, accum_lane_pos - dist,  s=1, color="red")
    plt.show()
    plt.scatter(dist, accum_lane_pos - dist, s=1, color="blue")
    plt.show()
        
            
    

                
                
                

        
            
    
    
            
        
        
        
        
    
    



if __name__ == "__main__":
    
    zoneShape = ['25003401-AddedOnRampEdge_0', 2000, 100]
    detector = detectionZone(zoneShape)
    detectionZoneList = detector.detectionZoneList
    sim = Simulation()
    sim.Run(detector = zoneShape)
    tresDict = sim.detected_trespassing
    
    
    
    
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
    
    

