#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 20:52:09 2020

@author: riasatislam
"""
import pandas as pd
import numpy as np
from scipy import signal

import JointAngles_IMU_MA
import StandingCalib_IMU_MA

# Input Data
staticQuaternions = pd.read_csv('staticQuaternions.csv',header=None)
# main.m:2
walkingQuaternions = pd.read_csv('walkingQuaternions.csv',header=None)
# main.m:3
staticPelvis_Orientation[1] = staticQuaternions.iloc[:,0:4]
# main.m:5
staticThigh_Orientation[1] = staticQuaternions.iloc[:,16:20]
# main.m:6
staticThigh_Orientation[2] = staticQuaternions.iloc[:,4:8]
# main.m:7
staticShank_Orientation[1] = staticQuaternions.iloc[:,20:24]
# main.m:8
staticShank_Orientation[2] = staticQuaternions.iloc[:,8:12]
# main.m:9
staticFoot_Orientation[1] = staticQuaternions.iloc[:,24:28]
# main.m:10
staticFoot_Orientation[2] = staticQuaternions.iloc[:,12:16]
# main.m:11
walkingPelvis_Orientation[1] = walkingQuaternions.iloc[:,0:4]
# main.m:13
walkingThigh_Orientation[1] = walkingQuaternions.iloc[:,16:20]
# main.m:14
walkingThigh_Orientation[2] = walkingQuaternions.iloc[:,4:8]
# main.m:15
walkingShank_Orientation[1] = walkingQuaternions.iloc[:,20:24]
# main.m:16
walkingShank_Orientation[2] = walkingQuaternions.iloc[:,8:12]
# main.m:17
walkingFoot_Orientation[1] = walkingQuaternions.iloc[:,24:28]
# main.m:18
walkingFoot_Orientation[2] = walkingQuaternions.iloc[:,12:16]
# main.m:19
# Filter data

Wn = 60 / 2
# main.m:23

cutoff = 6
# main.m:24
b, a = signal.butter(2, cutoff / Wn, 'low')
# main.m:25
for side in np.arange(1, 2).np.reshape(-1):
        static_pelvis_standing = signal.filtfilt(b, a, (staticPelvis_Orientation[1, 1]))
# main.m:29
        static_rthigh_standing[side] = signal.filtfilt(b, a, (staticThigh_Orientation[1, side]))
# main.m:30
        static_rshank_standing[side] = signal.filtfilt(b, a, (staticShank_Orientation[1, side]))
# main.m:31
        static_rfoot_standing[side] = signal.filtfilt(b, a, (staticFoot_Orientation[1, side]))
# main.m:32
        walking_pelvis_standing = signal.filtfilt(b, a, (walkingPelvis_Orientation[1, 1]))
# main.m:34
        walking_rthigh_standing[side] = signal.filtfilt(b, a, (walkingThigh_Orientation[1, side]))
# main.m:35
        walking_rshank_standing[side] = signal.filtfilt(b, a, (walkingShank_Orientation[1, side]))
# main.m:36
        walking_rfoot_standing[side] = signal.filtfilt(b, a, (walkingFoot_Orientation[1, side]))
# main.m:37

    # Calculating Joint Angles
d1 = copy(static_pelvis_standing)
# main.m:41
d2 = copy(static_rthigh_standing)
# main.m:42
d3 = copy(static_rshank_standing)
# main.m:43
d4 = copy(static_rfoot_standing)
# main.m:44
d5 = copy(walking_pelvis_standing)
# main.m:46
d6 = copy(walking_rthigh_standing)
# main.m:47
d7 = copy(walking_rshank_standing)
# main.m:48
d8 = copy(walking_rfoot_standing)
# main.m:49
for side in np.arange(1, 2).np.reshape(-1):
        q_imu_pv_bf, q_imu_th_bf, q_imu_sh_bf, q_imu_ft_bf = StandingCalib_IMU_MA(d1, d2[1, side], d3[1, side], d4[1, side], 5, 50, nargout=4)
# main.m:53
        Ahip_angle[side], Aknee_angle[side], Aankle_angle[side] = JointAngles_IMU_MA(q_imu_pv_bf, q_imu_th_bf, q_imu_sh_bf, q_imu_ft_bf, d5, d6[1, side], d7[1, side], d8[1, side], 1, nargout=3)
# main.m:55

    # output data as csv file

M = np.concatenate([Aankle_angle[1], Aankle_angle[2], Ahip_angle[1],Ahip_angle[2], Aknee_angle[1], Aknee_angle[2]])
# main.m:62
#filename = sprintf('jointangles_%s.csv', datestr(now, 'ddmmyyyy_HHMMSS'))
# main.m:63
# csvwrite(filename, M)
#     # Plotting for testing purposes
# plot(Aknee_angle[1, 2](np.arange(), 1))
# hold('on')
# plot(Aknee_angle[1, 1](np.arange(), 1), 'r')
