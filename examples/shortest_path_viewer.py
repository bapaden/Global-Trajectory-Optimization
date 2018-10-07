import numpy as np
import matplotlib.pyplot as plt
from numpy import genfromtxt
from matplotlib import cm
from matplotlib.patches import Circle



### Shortest path demo

#Load results from txt file
path = np.transpose(np.genfromtxt('../build/examples/shortest_path_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('../build/examples/shortest_path_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(10,10))
ax = plt.gca()
ax.set_xlim([-1,12])
ax.set_ylim([-1,12])
#Solution from planner
plt.plot(path[0],path[1])
#Nodes of search tree
plt.scatter(nodes[0],nodes[1],s=0.5)

ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([10.0, 10.0],0.5, facecolor="green",alpha=0.3,edgecolor="none"))
plt.show()



### Nonholonomic car demo

#Load results from txt file
path = np.transpose(np.genfromtxt('../build/examples/nonholonomic_car_demo.txt', delimiter=','))
nodes = np.transpose(np.genfromtxt('../build/examples/nonholonomic_car_demo_nodes.txt', delimiter=','))

#Plot data
fig = plt.figure(figsize=(10,10))
ax = plt.gca()
ax.set_xlim([-1,12])
ax.set_ylim([-1,12])
#Solution from planner
plt.plot(path[0],path[1])
#Nodes of search tree
plt.scatter(nodes[0],nodes[1],s=0.5)

ax.add_patch(Circle([3.0, 2.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([6.0, 8.0],2.0, facecolor="black",alpha=0.7,edgecolor="none"))
ax.add_patch(Circle([10.0, 10.0],0.5, facecolor="green",alpha=0.3,edgecolor="none"))
plt.show()

