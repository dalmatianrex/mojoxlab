#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 18:24:23 2020

@author: riasatislam
"""
import numpy as np
from dcm import _direction_cosine_matrix
from pytest import approx

def test__direction_cosine_matrix():
    a = _direction_cosine_matrix([0,0.200000000000000,0.400000000000000,0.600000000000000])
    b = np.array([[-0.85714286,  0.28571429,  0.42857143],[ 0.28571429, -0.42857143,  0.85714286],[ 0.42857143,  0.85714286,  0.28571429]])
    assert a == approx(b)