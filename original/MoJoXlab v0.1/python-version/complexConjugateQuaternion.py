import numpy as np
def complexConjugateQuaternion(q=None,*args,**kwargs):
    cc=np.transpose(np.concatenate([q(1),- q(2),- q(3),- q(4)]))
    return cc