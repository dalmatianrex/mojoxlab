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