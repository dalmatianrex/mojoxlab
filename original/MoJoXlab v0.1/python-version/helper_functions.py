#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 20:52:09 2020

@author: riasatislam
"""
import numpy as np
import math

def correctionQuaternion(p=None,*args,**kwargs):
    q0=p(1)
    q1=p(2)
    q2=p(3)
    q3=p(4)  
    theta=math.acos(np.dot(2,(np.dot(q1,q3) + np.dot(q0,q2))))
    N=np.concatenate([np.dot(2,(np.dot(q1,q2) + np.dot(q0,q3))),q2 ** 2 + q3 ** 2 - q0 ** 2 - q1 ** 2,0])
    n1=np.transpose(N)
    n1=n1 / np.norm(n1)    
    q=np.transpose(np.concatenate([math.cos(theta / 2),np.dot(n1(1,1),math.sin(theta / 2)),np.dot(n1(2,1),math.sin(theta / 2)),np.dot(n1(3,1),math.sin(theta / 2))]))
    return q

def BFtoGF(qimu_bf_b_bf=None,q_bf_b=None,*args,**kwargs):
    cc=complexConjugateQuaternion(q_bf_b)
    gf=multiplicationQuaternions(qimu_bf_b_bf,cc)
    gf=gf / np.norm(gf)
    return gf

def complexConjugateQuaternion(q=None,*args,**kwargs):
    cc=np.transpose(np.concatenate([q(1),- q(2),- q(3),- q(4)])) #this line might need fixing
    return cc

def multiplicationQuaternions(x=None, y=None, *args, **kwargs):
    fi = np.dot(x(1), y(1)) - np.dot(x(2), y(2)) - np.dot(x(3), y(3)) - np.dot(x(4), y(4))
    se = np.dot(x(1), y(2)) + np.dot(x(2), y(1)) + np.dot(x(3), y(4)) - np.dot(x(4), y(3))
    th = np.dot(x(1), y(3)) - np.dot(x(2), y(4)) + np.dot(x(3), y(1)) + np.dot(x(4), y(2))
    fo = np.dot(x(1), y(4)) + np.dot(x(2), y(3)) - np.dot(x(3), y(2)) + np.dot(x(4), y(1))
    mq = np.transpose(np.concatenate([fi, se, th, fo]))
    mq = mq / np.norm(mq)
    return mq

def e2B(x=None,y=None,*args,**kwargs):
    #Floating axis
    c=np.cross(x,y)
    e2=c / np.norm(c)
    return e2

def GFtoBF(q_bf_b=None,qimu_b=None,*args,**kwargs):
    #the sensor-to-body orientation
    cc=complexConjugateQuaternion(q_bf_b)
    bf=multiplicationQuaternions(cc,qimu_b)
    bf=bf / np.norm(bf)
    return bf

def DCM(q=None,*args,**kwargs):
    #convert from unit quaternions to direction cosine matrix
    q=q / np.norm(q)
    M=np.concatenate([[q(1) ** 2 + q(2) ** 2 - q(3) ** 2 - q(4) ** 2,np.dot(2,(np.dot(q(2),q(3)) - np.dot(q(1),q(4)))),np.dot(2,(np.dot(q(2),q(4)) + np.dot(q(1),q(3))))],[np.dot(2,(np.dot(q(2),q(3)) + np.dot(q(1),q(4)))),q(1) ** 2 - q(2) ** 2 + q(3) ** 2 - q(4) ** 2,np.dot(2,(np.dot(q(3),q(4)) - np.dot(q(1),q(2))))],[np.dot(2,(np.dot(q(2),q(4)) - np.dot(q(1),q(3)))),np.dot(2,(np.dot(q(3),q(4)) + np.dot(q(1),q(2)))),q(1) ** 2 - q(2) ** 2 - q(3) ** 2 + q(4) ** 2]])
    return M