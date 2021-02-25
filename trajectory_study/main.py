"""
Title: main file for trajectory generation and adaptation
"""

import matplotlib.pyplot as plt
import numpy as np


# set environment parameters
area_center_x = 10
area_center_y = 10
area_edge = 5
area_contour_x = [area_center_x-area_edge, area_center_x+area_edge, area_center_x+area_edge, area_center_x-area_edge,
                  area_center_x-area_edge]
area_contour_y = [area_center_y-area_edge, area_center_y-area_edge, area_center_y+area_edge, area_center_y+area_edge,
                  area_center_y-area_edge,]


# plot the whole thing

plt.figure()

# plot area for flight
plt.plot(area_contour_x, area_contour_y, color="r", markersize=5)

plt.show()

