# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np

def Plot_Dist_Error_Distribution(tripsDict):
    accum_lane_pos = np.asarray(
            [tripsDict[_key]['accum_lane_pos'] for _key in tripsDict ])
    dist = np.asarray(
            [tripsDict[_key]['distance'] for _key in tripsDict ])
    timespan = np.asarray(
            [tripsDict[_key]['timespan'] for _key in tripsDict ])
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


def plot_states(states, zonename):
    _flo = states[:,1]
    _den = states[:,0]
    plt.scatter(_den, _flo, s=1, color="black")
    plt.xlabel("density veh/km")
    plt.ylabel("flow veh/h")
    plt.xlim(0, 140)
    plt.ylim(0, 2000)
    plt.title(zonename)
    plt.show()