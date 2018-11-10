#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
import math

with open('sphere2500-g2o', 'r') as f:
    with open('sphere2500-g2o-output-edges.txt', 'w+') as g:
        while True:
            x = f.readline()
            if not x: break

            data = x.split()
            if(data[0][0] == 'E'):
                g.write(x)


N = 0
error = 0.0
error_abs = 0.0
with open('sphere2500-g2o-output-edges.txt', 'r') as f:
        while True:
            x = f.readline()
            if not x: break

            data_out = x.split()
            node1 = float(data_out[1])
            node2 = float(data_out[2])
            with open('sphere2500-ground-truth.txt', 'r') as g:
                while True:
                    y = g.readline()
                    if not y: break

                    data_gt = y.split()
                    nodegt_1 = float(data_gt[1])
                    nodegt_2 = float(data_gt[2])
                    if(nodegt_1 == node1 and nodegt_2 == node2):
                        N = N + 1
                        break
            
            error = error + ( float(data_gt[3]) - float(data_out[3]) )**2 + ( float(data_gt[4]) - float(data_out[4]) )**2 + ( float(data_gt[5]) - float(data_out[5]) )**2 + ( float(data_gt[6]) - float(data_out[6]) )**2 + ( float(data_gt[7]) - float(data_out[7]) )**2 + ( float(data_gt[8]) - float(data_out[8]) )**2
            error_abs = error_abs + abs( float(data_gt[3]) - float(data_out[3]) ) + abs( float(data_gt[4]) - float(data_out[4]) ) + abs( float(data_gt[5]) - float(data_out[5]) ) + abs( float(data_gt[6]) - float(data_out[6]) ) + abs( float(data_gt[7]) - float(data_out[7]) ) + abs( float(data_gt[8]) - float(data_out[8]) )
print(math.sqrt(error/N))
print(error_abs/N)

