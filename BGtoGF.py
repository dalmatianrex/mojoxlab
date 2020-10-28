import numpy as np
def BFtoGF(qimu_bf_b_bf=None,q_bf_b=None,*args,**kwargs):
    cc=complexConjugateQuaternion(q_bf_b)
    gf=multiplicationQuaternions(qimu_bf_b_bf,cc)
    gf=gf / np.norm(gf)
    return gf