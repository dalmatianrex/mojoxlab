#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python version:
Created on Mon Jan  6 20:52:09 2020

@author: riasatislam

Matlab version:
    #Written by MOhammad Al-Amri, Cardiff University, in August 2017
    
    #Input: sensor orientation data from a period of quiet stance. Orientation
#data are required from sensors placed on the pelvis, thigh and shank.
#Orientation data are optional from sensors placed on the foot. The period
#of quiet stance is defined by StartStance and EndStance.
    
    #Output: rotation matrices to calibrate sensor to segment orientation
    
    #--------------------------------------------------------------------------
    
    #Body frame quaternions obtained during calibration posture
# [ q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf]
"""
import statistics as stat
import numpy as np
import math

from helper_functions import * 

def StandingCalib_IMU_MA(Pelvis_Orientation=None,Thigh_Orientation=None,Shank_Orientation=None,Foot_Orientation=None,StanceStart=None,StanceEnd=None,*args,**kwargs):
    q_imu_f_pv0=stat.median(Pelvis_Orientation(np.arange(StanceStart,StanceEnd),np.arange())).T
# StandingCalib_IMU_MA.m:18
    q_imu_f_th0=stat.median(Thigh_Orientation(np.arange(StanceStart,StanceEnd),np.arange())).T
# StandingCalib_IMU_MA.m:19
    q_imu_f_sh0=stat.median(Shank_Orientation(np.arange(StanceStart,StanceEnd),np.arange())).T
# StandingCalib_IMU_MA.m:20
    q_imu_f_ft0=stat.median(Foot_Orientation(np.arange(StanceStart,StanceEnd),np.arange())).T
# StandingCalib_IMU_MA.m:22
    #--------------------------------------------------------------------------
#Calculate the body-to-sensor rotation matrix
    
    #pelvis
    
    #convert from unit quaternion to Direction Cosine Matrix
    qc_pv=correctionQuaternion(q_imu_f_pv0)
# StandingCalib_IMU_MA.m:31
    
    #Initial Quaternion Definition body quaternions during calibration posture
    i=np.concatenate([1,0,0])
# StandingCalib_IMU_MA.m:34
    
    j=np.concatenate([0,1,0])
# StandingCalib_IMU_MA.m:35
    
    k=np.concatenate([0,0,1])
# StandingCalib_IMU_MA.m:36
    
    rad_90=np.deg2rad(90)
# StandingCalib_IMU_MA.m:38
    rad_180=np.deg2rad(180)
# StandingCalib_IMU_MA.m:39
    q_rot90=np.transpose(np.concatenate([math.cos(rad_90 / 2),np.dot(1,math.sin(rad_90 / 2)),0,0]))
# StandingCalib_IMU_MA.m:40
    
    q_rot180=np.transpose(np.concatenate([math.cos(rad_180 / 2),np.dot((math.sqrt(2) / 2),math.sin(rad_180 / 2)),np.dot(0(math.sqrt(2) / 2),math.sin(rad_180 / 2))]))
# StandingCalib_IMU_MA.m:41
    
    #The body frame of the pelvis with respect to the global frame during the initial posture
    q_bf_pv0=multiplicationQuaternions(qc_pv,q_imu_f_pv0)
# StandingCalib_IMU_MA.m:44
    #Definition of body frame of the thigh
    q_bf_th0=multiplicationQuaternions(q_bf_pv0,q_rot90)
# StandingCalib_IMU_MA.m:47
    #Definition of body frame of the shank
    q_bf_sh0=copy(q_bf_th0)
# StandingCalib_IMU_MA.m:50
    #Definition of body frame of the foot
    
    q_bf_ft0=multiplicationQuaternions(q_bf_sh0,q_rot180)
# StandingCalib_IMU_MA.m:54
    # sensor-to-body orientation (Equation (7))
    q_imu_pv_bf=GFtoBF(q_bf_pv0,q_imu_f_pv0)
# StandingCalib_IMU_MA.m:58
    
    q_imu_th_bf=GFtoBF(q_bf_th0,q_imu_f_th0)
# StandingCalib_IMU_MA.m:59
    
    q_imu_sh_bf=GFtoBF(q_bf_sh0,q_imu_f_sh0)
# StandingCalib_IMU_MA.m:60
    
    q_imu_ft_bf=GFtoBF(q_bf_ft0,q_imu_f_ft0)
# StandingCalib_IMU_MA.m:61
    
    return q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf