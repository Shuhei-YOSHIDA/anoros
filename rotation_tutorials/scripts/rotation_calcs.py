#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Shuhei-YOSHIDA
# 参考: 
# www.mss.co.jp/technology/report/pdf/19-08.pdf
# "MATLABによるクォータニオン数値計算"   

# memo: 
# dcm -方向余弦行列，回転行列
# z-y-x オイラー角
# クォータニオンx,y,z,wの順
# 微分方程式はscipyで．
# mainではテスト？

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
    q2 = np.array(q)*np.array(q)
    dcm = np.array([[q2[0]-q2[1]-q2[2]+q2[3], 2*(q[0]*q[1]+q[2]*q[3]), 2*(q[0]*q[2]-q[1]*q[3])], \
                    [2*(q[0]*q[1]-q[2]*q[3]), q2[1]-q2[0]-q2[2]+q2[3], 2*(q[1]*q[2]+q[0]*q[3])], \
                    [2*(q[0]*q[2]+q[1]*q[3]), 2*(q[1]*q[2]-q[0]*q[3]), q2[2]-q2[0]-q2[1]+q2[3]]  ])
    return dcm

def dcm2eular(dcm):
    eular = np.array([np.arctan2(dcm[1][2], dcm[2][2])], \
                     [np.arctan2(-dcm[0][2], np.sqrt(dcm[1][2]**2+dcm[2][2]**2))], \
                     [np.arctan2(dcm[0][1], dcm[0][0])]);

    return eular
    
def dcm2quat(dcm):
    q = np.array([np.sqrt(1+dcm[0][0]-dcm[1][1]-dcm[2][2])/2, \
                  np.sqrt(1-dcm[0][0]+dcm[1][1]-dcm[2][2])/2, \
                  np.sqrt(1-dcm[0][0]-dcm[1][1]+dcm[2][2])/2, \
                  np.sqrt(1+dcm[0][0]+dcm[1][1]+dcm[2][2])/2 ])
    idx = np.argmax(q)

    if idx == 0:
        [q[1], q[2], q[3]] = np.array(0.25/q[0]*[dcm[0][1]+dcm[1][0], dcm[0][2]+dcm[2][0], dcm[1][2]-dcm[2][1]])
    elif idx == 1:
        [q[0], q[2], q[3]] = np.array(0.25/q[1]*[dcm[0][1]+dcm[1][0], dcm[2][1]+dcm[1][2], dcm[2][0]-dcm[0][2]])
    elif idx == 2:
        [q[0], q[1], q[3]] = np.array(0.25/q[2]*[dcm[2][0]+dcm[0][2], dcm[2][1]+dcm[1][2], dcm[0][1]-dcm[1][0]])
    elif idx == 3:
        [q[0], q[1], q[2]] = np.array(0.25/q[3]*[dcm[1][2]-dcm[2][1], dcm[2][0]-dcm[0][2], dcm[0][1]-dcm[1][0]])

    return quat
    

# オイラー角の微分方程式
def eular_eq(t, x, rate):
    # dx : オイラー角の時間微分
    # t, x, rate: 時間，オイラー角, 角速度

    # オイラー角
    phi = x[0]
    the = x[1]
    psi = x[2]
    # 角速度
    p = rate[0]
    q = rate[1]
    r = rate[2]
    sp = np.sin(phi)
    cp = np.cos(phi)

    # 微分方程式 theta=+90,-90で特異点
    dx = np.array([p+(q*sp+r*cp)*np.tan(the)), \
                   q*cp-r*sp, \
                   (q*cp+r*cp)/np.cos(the)])

    return dx

# クォータニオンの微分方程式
def quat_eq(t, x, rate):
    # dx : クォータニオンの時間微分
    # t, x, rate: 時間，クォータニオン，角速度

    # 角速度
    p = rate[0]
    q = rate[1]
    r = rate[2]
    
    mat = np.array(0.5*[[0, r, -q, p], \
                        [-r, 0, p, q], \
                        [q, -p, 0, r], \
                        [-p, -q, -r, 0]])

    dx = mat.dot(x)

    return dx

if  __name__ == '__main__':
    print 'rotation_calc is not completed'
    eular = np.array([0, 0, 0])
    print eular2dcm(eular)
