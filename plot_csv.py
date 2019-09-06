# -*- coding: utf-8 -*-
"""
Created on Wed Jul 17 16:22:54 2019

@author: 726094
"""

import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.patches as patches
import numpy as np
import matplotlib.cm as cm

x=[]
y=[]
x_local = []
y_local = []
center_x =[]
center_y = []
x1arr = []
y1arr = []

file = pd.read_csv('graph_data.csv', delimiter = ',')

x1 = file.global_position_x
y1 = file.global_position_y 
pathx = file.path_x
pathy = file.path_y
cols = int((file.shape[1] - 15)/4)
num_obstacles = file.num_obstacles[0]

for i in range(0, 100):
    p = 'Particle_x'+str(i+1)
    q = 'Particle_y'+str(i+1)
    x.append(file[p])
    y.append(file[q])

for i in range(0, 100):
    p = 'local_position_x'+str(i+1)
    q = 'local_position_y'+str(i+1)
    x_local.append(file[p])
    y_local.append(file[q])

for i in range(0, num_obstacles):
    p = 'center_x'+str(i+1)
    q = 'center_y'+str(i+1)
    r = 'x1'+str(i+1)
    s = 'y1'+str(i+1)
    center_x.append(file[p])
    center_y.append(file[q])
#    x1arr.append(file[r])
#    y1arr.append(file[s])

plt.figure(figsize=(13,13))


"""Plotting Obstacles"""
fig = plt.gcf()
ax = fig.gca()
for i in range(num_obstacles):
    circ = patches.Circle((center_x[i][0], center_y[i][0]), file.r[0], linewidth=5,edgecolor='r',facecolor='none')
    ax.add_patch(circ)
    
#    rect = patches.Rectangle((x1arr[i][0],y1arr[i][0]),file.side[0],file.side[0],linewidth=5,edgecolor='b',facecolor='none')
#    ax.add_patch(rect)

"""Plotting global positions"""
plt.plot(x1,y1, marker='o')
#plt.plot(x[0],y[0], marker='o')
    
"""Plotting particle positions"""
for i in range(0, 100):
#    Line Plot
#    plt.plot(x[i],y[i], marker='o')
#    Intensity plot
    colors = cm.rainbow(np.linspace(0, 1, len(x[i])))
    for c in zip(x[i], colors):
        plt.scatter(x[i], y[i], color=colors)

"""PLotting local best positions"""
#for i in range(0, 100):
#    plt.plot(x_local[i],y_local[i], marker='o')

"""PLotting the path"""
#plt.plot(pathx,pathy, marker='o')

plt.ylim(-1700, 1700)
plt.xlim(-1700, 1700)
#plt.ylim(-5700, 5700)
#plt.xlim(-5700, 5700)

plt.title('Data from the CSV File: global, local, and particle positions')

plt.xlabel('x')
plt.ylabel('y')
plt.show()

#===========================================================================================#


