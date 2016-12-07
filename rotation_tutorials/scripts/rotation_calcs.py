#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Shuhei-YOSHIDA
# 参考: 
# www.mss.co.jp/technology/report/pdf/19-08.pdf
# "MATLABによるクォータニオン数値計算"   

# memo: 
# dcm -方向余弦行列，回転行列
# z-y-x オイラー角

import numpy as np

# オイラー角から方向余弦行列を導出
def eular2dcm(eular):
    s = np.sin(eular)
    c = np.cos(eular)
    dcm = np.array([[ c[1]*c[2],                 c[1]*s[2],                -s[1]],      \
                    [-c[0]*s[2]+s[0]*s[1]*c[2],  c[0]*c[2]+s[0]*s[1]*s[2],  s[0]*c[1]], \
                    [ s[0]*s[2]+c[0]*s[1]*c[2], -s[0]*c[2]+c[0]*s[1]*s[2],  c[0]*c[1]]  ])
    return dcm

# クォータニオンから
def quat2dcm(q):
    print "not yet"
    
def dcm2eular(dcm):
    print "not yet"
    
def dcm2quat(dcm):
    print "not yet"
    
# オイラー角の微分方程式
def eular_eq(t, x):
    print "not yet"
    
# クォータニオンの微分方程式
def quat_eq(t, x):
    print "not yet"
    

if  __name__ == '__main__':
    print 'rotation_calc is not completed'
    eular = np.array([0, 0, 0])
    print eular2dcm(eular)
