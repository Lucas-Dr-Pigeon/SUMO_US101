# -*- coding: utf-8 -*-
import os, sys
import traci
import tqdm
import traci.constants as tc
import copy
import math
import numpy as np
import sumo

class detectionZone(sumo.Sumo):
    def init_zones(self):
        _sumLen = 0
        _LaneList = []
        _dictOfDict = {}
        thisLen = traci.lane.getLength(self.startLane)
        startLanePos = min(self.startLanePos, thisLen)
        if self.not_segment(self.startLane):
            raise ("The starting lane was on a ramp!")
        
        ''' initiate the order of lanes to 0 '''
        _index = 0
        
        ''' first lane is long enough? first lane isn't long enough? ''' 
        if self.zoneLength <= thisLen - startLanePos:
            _LaneList.append([startLanePos, startLanePos + self.zoneLength, self.startLane])
            _dictOfDict[self.startLane] = _LaneList
        else:
            _connLanes = traci.lane.getLinks(self.startLane)
            _LaneList.append([startLanePos, thisLen, self.startLane])
            _sumLen += thisLen - startLanePos
            _ = self.get_next_connected_lane(_connLanes, _LaneList, _sumLen, _dictOfDict, self.zoneLength, _index)
        return _dictOfDict
    
    def segment_list_to_dict(self):
        _dict = {}
        for i, segment in enumerate(self.detectionZoneList):
            _dict[segment[2]] = {"index": i, "start": segment[0], "end": segment[1]}
        return _dict
    
    def __init__(self, shape, mode="sectioned", window=10):
        super().__init__()
        self.startLane, self.zoneLength, self.startLanePos = shape
        self.segmentDict = self.init_zones()
        ''' this should be changed later '''        
        self.coveredEdges = [zone[2] for zone in list(self.segmentDict.values())[0]] # changes need to be made
        ''' this should be changed later '''
        self.detectionZoneList = list(self.segmentDict.values())[0] # changes need to be made
        self.detectionZoneDict = self.segment_list_to_dict()
        self.time = 0
        ''' mode = "sliding" or "sectioned" '''
        self.mode = mode
        self.window = int(window/self.step_length)
        ''' store detected traffic states '''
        self.allstates = np.zeros(shape=(0,5))
    
    def get_all_trespassing(self, AllSubscriptions):
        self.trespassing = self.CONCATE_VEHICLE_SUB(AllSubscriptions)
        return self.trespassing
        
    def not_segment(self, thisLane):
        edge = traci.lane.getEdgeID(thisLane)
        LaneNo = traci.edge.getLaneNumber(edge)
        if LaneNo < 2:
            return True
        
    def get_next_connected_lane(self, connLanes, LaneList, sumLen, dictOfDict, target_length, index):
        for _next in connLanes:
            _nextLane = _next[0]
            thisLen = traci.lane.getLength(_nextLane)
            if self.not_segment(_nextLane):
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
                self.get_next_connected_lane(_connLanes, _LaneList, _sumLen, dictOfDict, target_length, _index)
    
    def CONCATE_VEHICLE_SUB(self, subList):
        _subList = copy.deepcopy(subList)
        null_trip = {"distance":0, "timespan":0, "accum_lane_pos":0, "trespass":0}
        tripsDict = {}
        for timestamp in range(0, len(_subList)):
            if not timestamp < len(subList)-1:
                continue
            for vehID in _subList[timestamp]:
                if _subList[timestamp][vehID] != "checked":
                    if not self.IF_DETECTED(timestamp, vehID, subList):
                        continue
                    tripsDict[(vehID, timestamp)] = copy.deepcopy(null_trip)
                    self.UPDATE_CHECK(tripsDict[(vehID, timestamp)], _subList, timestamp, vehID)
                    if tripsDict[(vehID, timestamp)] == null_trip:
                        del tripsDict[(vehID, timestamp)]
                    _subList[timestamp][vehID] = "checked"
        return tripsDict    
        
    def UPDATE_CHECK(self, tripInfo, subList, timestamp, vehID):
        if vehID in subList[timestamp+1]:
            if self.IF_DETECTED(timestamp+1, vehID, subList):
                tripInfo["distance"] += subList[timestamp+1][vehID][132] - subList[timestamp][vehID][132]
                tripInfo["timespan"] += 1
                tripInfo["accum_lane_pos"] += self.GET_ACCUMULATED_LANE_POSITION(laneID=subList[timestamp+1][vehID][81], lanePos=subList[timestamp+1][vehID][86]) \
                    - self.GET_ACCUMULATED_LANE_POSITION(laneID=subList[timestamp][vehID][81], lanePos=subList[timestamp][vehID][86])
                ''' if trespass ? '''
                if subList[timestamp][vehID][81] != subList[timestamp+1][vehID][81]:
                    tripInfo["trespass"] += 1
                ''' to prevent keyerror '''
                if timestamp+1 < len(subList)-1:
                    self.UPDATE_CHECK(tripInfo, subList, timestamp+1, vehID)
            subList[timestamp+1][vehID]= "checked"

    def IF_DETECTED(self, timestamp:int, vehID:str, subList)->bool:
        laneID = subList[timestamp][vehID][81]
        lanePos = subList[timestamp][vehID][86]
        # try:
        #     if self.detectionZoneDict[laneID]['start'] < lanePos <= self.detectionZoneDict[laneID]['end']:
        #         return True
        #     else:
        #         return False
        # except KeyError:
        #     print (
        #         ("Unexpected lane ID: %s caused by vehicle %s at time %s!") %
        #         (laneID, vehID, timestamp)
        #         )
        if  laneID not in self.detectionZoneDict.keys():
            return False
        if not self.detectionZoneDict[laneID]['start'] < lanePos <= self.detectionZoneDict[laneID]['end']:
            return False
        else:
            return True  
        
    def GET_ACCUMULATED_LANE_POSITION(self, laneID, lanePos):
        if self.detectionZoneDict[laneID]["index"] == 0:
            accumLanePos = lanePos - self.detectionZoneDict[laneID]["start"]
        else:
            _index = self.detectionZoneDict[laneID]["index"]
            accumLanePos = lanePos
            for i in range(_index):
                accumLanePos += self.detectionZoneList[i][1] - self.detectionZoneList[i][0]
        return accumLanePos
    
    def scan(self, vehicleSub, subscriptionResults):
        for laneID in self.coveredEdges:
            vehDetectList = traci.lane.getLastStepVehicleIDs(laneID)
            for vehID in vehicleSub:
                if traci.vehicle.getLaneID(vehID) not in self.coveredEdges:
                    vehicleSub.remove(vehID)
                    traci.vehicle.unsubscribe(vehID)
            for vehID in vehDetectList:
                if vehID not in vehicleSub:
                    vehicleSub.append(vehID)
                    traci.vehicle.subscribe(vehID, (tc.VAR_LANEPOSITION, tc.VAR_DISTANCE, tc.VAR_LANE_ID))
        subscriptionResults.append(
            copy.deepcopy(traci.vehicle.getAllSubscriptionResults())
            )
        # print (vehicleSub)
        if self.mode == "sliding":
            self.get_last_traffic_states(subscriptionResults)
        elif self.mode == "sectioned":
            time = traci.simulation.getTime()
            if not time % int(self.window):
                self.get_last_traffic_states(subscriptionResults)
    
    def get_last_traffic_states(self, subscriptions):
        time = traci.simulation.getTime()
        # print (time,self.window)
        if time < self.window:
            return 
        # print (subscriptions)
        tresDict = self.CONCATE_VEHICLE_SUB(subscriptions[-self.window:])
        tresArray = np.asarray([ list(tresDict[tresID].values()) for tresID in tresDict ] )
        # print ("tres:", tresArray)
        # print (tresArray.shape)
        if tresArray.shape[0] == 0:
            return np.asarray([[0, 0, 0, time, time+self.window*self.step_length]])
        totalTresDistance = np.sum(tresArray[:,0])/1000
        totalTresTime = np.sum(tresArray[:,1])/3600*self.step_length
        zoneDensity = totalTresTime / (self.zoneLength/1000) / (self.window/3600*self.step_length)
        zoneFlow = totalTresDistance / (self.zoneLength/1000) / (self.window/3600*self.step_length)
        ''' update state history '''
        _state = np.asarray([[zoneDensity, zoneFlow, zoneFlow/zoneDensity, time, time+self.window*self.step_length]])
        # print (self.allstates.shape, _state.shape)
        self.allstates = np.concatenate((self.allstates, _state), axis=0)
        return _state
        
if __name__ == "__main__":

    detector = detectionZone(shape = ['25003401-AddedOnRampEdge_0', 2000, 100])
    
    A = np.asarray([[1,2,3]])
    B = np.zeros((0,3))
    print (A.shape, B.shape)
    AB = np.concatenate((A, B))
    C = np.asarray([[2,3,4]])
    ABC = np.concatenate((AB, C))
    print(ABC)
    