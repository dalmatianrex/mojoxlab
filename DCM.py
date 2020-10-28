#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  14 15:52:09 2020

@author: riasatislam
"""
import numpy as np

def _direction_cosine_matrix(q):
    """Returns direction cosine matrix from unit quaternions."""
    
    q = q / np.linalg.norm(q)
   
    m11 = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2 
    m12 = 2 * (q[1]*q[2] - q[0]*q[3]) 
    m13 = 2 * (q[1]*q[3] + q[0]*q[2])
    
    m21 = 2 * (q[1]*q[2] + q[0]*q[3]) 
    m22 = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2 
    m23 = 2 * (q[2]*q[3] - q[0]*q[1])
    
    m31 = 2 * (q[1]*q[3] - q[0]*q[2])
    m32 = 2 * (q[2]*q[3] + q[0]*q[1])
    m33 = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2 
    
    m = np.array([ [m11, m12, m13], [m21, m22, m23], [m31, m32, m33] ])
    
    return m
    
if __name__ == '__main__':
    q = [0,0.200000000000000,0.400000000000000,0.600000000000000]
    print(_direction_cosine_matrix(q))
    pass