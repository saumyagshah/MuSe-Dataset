#!/usr/bin/env python
# coding: utf-8

# In[24]:


import numpy as np
import math

with open('city10000-g2o', 'r') as f:
    with open('city10000-g2o-output-edges.txt', 'w+') as g:
        while True:
            x = f.readline()
            if not x: break

            data = x.split()
            if(data[0][0] == 'E'):
                g.write(x)


N = 0
error = 0.0
error_abs = 0.0
#Use commented code below for computing error metric for the Manhattan dataset
with open('city10000-g2o-output-edges.txt', 'r') as f:
        while True:
            x = f.readline()
            if not x: break

            data_out = x.split()
            node1 = float(data_out[1])
            node2 = float(data_out[2])
            with open('city10000-ground-truth.txt', 'r') as g:
                while True:
                    y = g.readline()
                    if not y: break

                    data_gt = y.split()
#                     nodegt_1 = float(data_gt[0])
                    nodegt_1 = float(data_gt[1])

#                     nodegt_2 = float(data_gt[1])
                    nodegt_2 = float(data_gt[2])

                    if(nodegt_1 == node1 and nodegt_2 == node2):
                        N = N + 1
                        break
            
#             error = error + ( float(data_gt[2]) - float(data_out[3]) )**2 + ( float(data_gt[3]) - float(data_out[4]) )**2 + ( float(data_gt[4]) - float(data_out[5]) )**2
            error = error + ( float(data_gt[3]) - float(data_out[3]) )**2 + ( float(data_gt[4]) - float(data_out[4]) )**2 + ( float(data_gt[5]) - float(data_out[5]) )**2

#             error_abs = error_abs + abs( float(data_gt[2]) - float(data_out[3]) ) + abs( float(data_gt[3]) - float(data_out[4]) ) + abs( float(data_gt[4]) - float(data_out[5]) )
            error_abs = error_abs + abs( float(data_gt[3]) - float(data_out[3]) ) + abs( float(data_gt[4]) - float(data_out[4]) ) + abs( float(data_gt[5]) - float(data_out[5]) )

