from neato_optical_flow import 
import numpy as np
import math

import rclpy
from rclpy.node import Node

field_of_view_x = 62.2
field_of_view_y = 48.8
height = 5

max_x_dist = 2 * math.cos(field_of_view_x/2) * height
max_y_dist = 2 * math.cos(field_of_view_y/2) * height

pixels_x = 1920
pixels_y = 1080
dist_per_pixel_x =max_x_dist/pixels_x
dist_per_pixel_y = max_y_dist/pixels_y

vector_displacement_list = []

def real_distance(vector_cloud_old, vector_cloud_new): # rename this appropriatly 
    for i in range(len(vector_cloud_ambigous))
        old_dist_x = old_vector_cloud[[i]][0] * dist_per_pixel_x # one vector in vector clouds x value and convert it to position in physical units
        old_dist_y = old_vector_cloud[[i]][1] * dist_per_pixel_y
        new_dist_x = new_vector_cloud[[i]][0] * dist_per_pixel_x 
        new_dist_y = new_vector_cloud[[i]][1] * dist_per_pixel_y

        old_distance = math.sqt((old_dist_x **2)+(old_dist_x **2)) # postion of vector in first image
        new_distance = math.sqt((new_dist_x **2)+(new_dist_x **2)) # position of vector in second image
        displacement[i] = new_distance - old_distance
    average_displacement = sum(displacement)/len(displacement)
return 
    print(average_displacement)
    