# -*- coding: utf-8 -*-
import networkx as nx
import scipy.stats
import numpy as np
import math

QUESTION = 2


if __name__ == "__main__":
    _dir = r"C:\Users\dachu\PycharmProjects\WyomingBSM\data\hollywood_freeway.shp"
    shp = nx.read_shp(_dir)
    
    
    if QUESTION == 2:
        p = scipy.stats.norm.sf(1.85)*2
        
        t = scipy.stats.t.ppf(1-0.1/2, 10)
        samples = np.asarray([0.92,2.23,1.71,2.58,1.72,0.83,1.87,0.62,2.12,1.68])
        print (np.mean(samples), np.std(samples), samples.shape[0])
        CI = [np.mean(samples) - t*np.std(samples)/math.sqrt(samples.shape[0]),
              np.mean(samples) + t*np.std(samples)/math.sqrt(samples.shape[0])]
        print ("CI: ",CI)
        
    if QUESTION == 3:
        samples = np.asarray([13.83, 15.24, 11.63, 10.65, 10.52, 13.29, 14.96, 10.54, 11.66, 12.22, 13.86, 11.78, 10.86, 13.06])
        mean = np.mean(samples)
        stdv = np.std(samples)
        cv = (stdv/mean)
        
        t3 = (mean - 11)/(stdv/math.sqrt(samples.shape[0]))
        p3 = scipy.stats.t.sf(t3, samples.shape[0]-1)*2
        
        newsamples = np.asarray([
            12.83, 14.24, 12.63, 10.65, 10.52, 13.29, 14.96, 10.54, 11.66, 12.22,
            13.86, 11.78, 10.86, 12.06, 11.53, 12.22, 11.93, 11.86, 12.17, 12.19,
            12.32, 12.05, 11.84, 12.30, 12.04, 11.61, 12.23, 12.10, 12.08, 11.89,
            12.14, 11.77, 11.88, 11.76, 11.87, 11.80, 12.10, 12.02, 12.21, 12.13,
            13.94, 14.26, 13.85, 14.17, 13.98, 14.10, 13.86, 13.87, 14.00, 14.04
            ])
        newmean = np.mean(newsamples)
        newstdv = np.std(newsamples)
        newcv = newstdv/newmean
        newz3 = (newmean - 12)/(newstdv/math.sqrt(50))

        newp3 = scipy.stats.norm.sf(abs(newz3))*2
        
    if QUESTION == 4:
        avgcv = np.mean(np.asarray([0.98, 0.85, 1.00, 1.07, 1,1])) 
        n_ = avgcv**2 * 2.58**2/ 0.01**2
        
    