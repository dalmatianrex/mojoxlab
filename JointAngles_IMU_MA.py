#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python version:
Created on Mon Jan  6 20:52:09 2020

@author: riasatislam

Matlab version:
    #Written by MOhammad Al-Amri, Cardiff University, in August 2017
    
# Pelvis_Orientation=Xsens_Pelvis_SensorOrientation_Quat{subj,task}{session,rater}{1,1};
# Thigh_Orientation=Xsens_UpperLeg_SensorOrientation_Quat{subj,task}{session,rater}{1,side};
# Shank_Orientation=Xsens_LowerLeg_SensorOrientation_Quat{subj,task}{session,rater}{1,side};
# Foot_Orientation=Xsens_Foot_SensorOrientation_Quat{subj,task}{session,rater}{1,side};
    
    #Written by MOhammad Al-Amri, Cardiff University, in August 2017
    
    #Script requires sensor orientation data from and outputs joint angles
    
    #Joint kinematics are based on Joint Coordinate System (JCS) as proposed by
#Grood and Suntay (1983)
    
    #--------------------------------------------------------------------------
"""
import numpy as np
import math

from helper_functions import * 

def JointAngles_IMU_MA(q_imu_pv_bf=None,q_imu_th_bf=None,q_imu_sh_bf=None,q_imu_ft_bf=None,Pelvis_Orientation=None,Thigh_Orientation=None,Shank_Orientation=None,Foot_Orientation=None,side=None,*args,**kwargs):
#Initial Quaternion Definition body quaternions during calibration posture
    i=np.concatenate([1,0,0])
# JointAngles_IMU_MA.m:17
    
    j=np.concatenate([0,1,0])
# JointAngles_IMU_MA.m:18
    
    k=np.concatenate([0,0,1])
# JointAngles_IMU_MA.m:19
    
    for x in np.arange(1,len(Pelvis_Orientation,1)).np.reshape(-1):
        # body to global orientation
        q_bf_pv[x,np.arange()]=BFtoGF(Pelvis_Orientation(x,np.arange()),q_imu_pv_bf)
# JointAngles_IMU_MA.m:25
        q_bf_th[x,np.arange()]=BFtoGF(Thigh_Orientation(x,np.arange()),q_imu_th_bf)
# JointAngles_IMU_MA.m:26
        q_bf_sh[x,np.arange()]=BFtoGF(Shank_Orientation(x,np.arange()),q_imu_sh_bf)
# JointAngles_IMU_MA.m:27
        if not q_imu_ft_bf:
            q_bf_ft[x,np.arange()]=BFtoGF(Foot_Orientation(x,np.arange()),q_imu_ft_bf)
# JointAngles_IMU_MA.m:29
        #################################
        # Floating axis of each Hip(h),Knee(k), and Ankle (a)
        e2_h[np.arange(),x]=e2B(np.dot(DCM(q_bf_th(x,np.arange())),i.T),np.dot(- DCM(q_bf_pv(x,np.arange())),j.T))
# JointAngles_IMU_MA.m:34
        e2_k[np.arange(),x]=e2B(np.dot(DCM(q_bf_sh(x,np.arange())),i.T),np.dot(DCM(q_bf_th(x,np.arange())),k.T))
# JointAngles_IMU_MA.m:35
        if not q_imu_ft_bf:
            e2_a[np.arange(),x]=e2B(np.dot(DCM(q_bf_ft(x,np.arange())),k.T),np.dot(DCM(q_bf_sh(x,np.arange())),k.T))
# JointAngles_IMU_MA.m:37
        #++++++++++++++++++++++++++++++++++++++++++++ 
    # Left Joint Angles 
    #++++++++++++++++++++++++++++++++++++++++++++
        if side == 1:
            #Hip Joint rotations
            alpa_hip[x]=math.asin(np.dot(e2_h(np.arange(),x),np.dot(DCM(q_bf_pv(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:44
            beta_hip[x]=math.pi / 2 - math.acos(np.dot(np.dot(- DCM(q_bf_pv(x,np.arange())),j.T),np.dot(DCM(q_bf_th(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:45
            gama_hip[x]=math.asin(np.dot(e2_h(np.arange(),x),np.dot(DCM(q_bf_th(x,np.arange())),k.T)))
# JointAngles_IMU_MA.m:46
            alpha_angle_hip[x]=np.rad2deg(alpa_hip(x))
# JointAngles_IMU_MA.m:47
            beta_angle_hip[x]=np.rad2deg(beta_hip(x))
# JointAngles_IMU_MA.m:48
            gama_angle_hip[x]=np.rad2deg(gama_hip(x))
# JointAngles_IMU_MA.m:49
            alpa_knee[x]=- math.asin(np.dot(e2_k(np.arange(),x),np.dot(DCM(q_bf_th(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:52
            beta_knee[x]=math.pi / 2 - math.acos(np.dot(np.dot(DCM(q_bf_th(x,np.arange())),k.T),np.dot(DCM(q_bf_sh(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:53
            gama_knee[x]=math.asin(np.dot(e2_k(np.arange(),x),np.dot(DCM(q_bf_sh(x,np.arange())),k.T)))
# JointAngles_IMU_MA.m:54
            alpha_angle_knee[x]=np.rad2deg(alpa_knee(x))
# JointAngles_IMU_MA.m:55
            beta_angle_knee[x]=np.rad2deg(beta_knee(x))
# JointAngles_IMU_MA.m:56
            gama_angle_knee[x]=np.rad2deg(gama_knee(x))
# JointAngles_IMU_MA.m:57
            if not q_imu_ft_bf:
                alpa_ankle[x]=math.asin(np.dot(e2_a(np.arange(),x),np.dot(DCM(q_bf_sh(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:61
                beta_ankle[x]=math.pi / 2 - math.acos(np.dot(np.dot(DCM(q_bf_sh(x,np.arange())),k.T),np.dot(DCM(q_bf_ft(x,np.arange())),k.T)))
# JointAngles_IMU_MA.m:62
                alpha_angle_ankle[x]=np.rad2deg(alpa_ankle(x))
# JointAngles_IMU_MA.m:63
                beta_angle_ankle[x]=np.rad2deg(beta_ankle(x))
# JointAngles_IMU_MA.m:64
            #++++++++++++++++++++++++++++++++++++++++++++ 
    # Right Joint Angles 
    #++++++++++++++++++++++++++++++++++++++++++++
        else:
            if side == 2:
                #Hip Joint rotations
                alpa_hip[x]=math.asin(np.dot(e2_h(np.arange(),x),np.dot(DCM(q_bf_pv(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:72
                beta_hip[x]=math.acos(np.dot(np.dot(- DCM(q_bf_pv(x,np.arange())),j.T),np.dot(DCM(q_bf_th(x,np.arange())),i.T))) - math.pi / 2
# JointAngles_IMU_MA.m:73
                gama_hip[x]=math.asin(np.dot(e2_h(np.arange(),x),np.dot(DCM(q_bf_th(x,np.arange())),k.T)))
# JointAngles_IMU_MA.m:74
                alpha_angle_hip[x]=np.rad2deg(alpa_hip(x))
# JointAngles_IMU_MA.m:75
                beta_angle_hip[x]=np.rad2deg(beta_hip(x))
# JointAngles_IMU_MA.m:76
                gama_angle_hip[x]=np.rad2deg(gama_hip(x))
# JointAngles_IMU_MA.m:77
                alpa_knee[x]=- math.asin(np.dot(e2_k(np.arange(),x),np.dot(DCM(q_bf_th(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:80
                beta_knee[x]=math.acos(np.dot(np.dot(DCM(q_bf_th(x,np.arange())),k.T),np.dot(DCM(q_bf_sh(x,np.arange())),i.T))) - math.pi / 2
# JointAngles_IMU_MA.m:81
                gama_knee[x]=math.asin(np.dot(e2_k(np.arange(),x),np.dot(DCM(q_bf_sh(x,np.arange())),k.T)))
# JointAngles_IMU_MA.m:82
                alpha_angle_knee[x]=np.rad2deg(alpa_knee(x))
# JointAngles_IMU_MA.m:83
                beta_angle_knee[x]=np.rad2deg(beta_knee(x))
# JointAngles_IMU_MA.m:84
                gama_angle_knee[x]=np.rad2deg(gama_knee(x))
# JointAngles_IMU_MA.m:85
                if not q_imu_ft_bf:
                    alpa_ankle[x]=math.asin(np.dot(e2_a(np.arange(),x),np.dot(DCM(q_bf_sh(x,np.arange())),i.T)))
# JointAngles_IMU_MA.m:89
                    beta_ankle[x]=math.acos(np.dot(np.dot(DCM(q_bf_sh(x,np.arange())),k.T),np.dot(DCM(q_bf_ft(x,np.arange())),k.T))) - math.pi / 2
# JointAngles_IMU_MA.m:90
                    alpha_angle_ankle[x]=np.rad2deg(alpa_ankle(x))
# JointAngles_IMU_MA.m:91
                    beta_angle_ankle[x]=np.rad2deg(beta_ankle(x))
# JointAngles_IMU_MA.m:92
    
    Hip_angle=np.concatenate([[alpha_angle_hip],[beta_angle_hip],[gama_angle_hip]]).T
# JointAngles_IMU_MA.m:96
    Knee_angle=np.concatenate([[alpha_angle_knee],[beta_angle_knee],[gama_angle_knee]]).T
# JointAngles_IMU_MA.m:97
    Ankle_angle=np.concatenate([[alpha_angle_ankle],[beta_angle_ankle]]).T
# JointAngles_IMU_MA.m:99
    return Hip_angle,Knee_angle,Ankle_angle