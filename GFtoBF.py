@function
def GFtoBF(q_bf_b=None,qimu_b=None,*args,**kwargs):
    varargin = GFtoBF.varargin
    nargin = GFtoBF.nargin

    #the sensor-to-body orientation
    cc=complexConjugateQuaternion(q_bf_b)
# GFtoBF.m:3
    bf=multiplicationQuaternions(cc,qimu_b)
# GFtoBF.m:4
    bf=bf / norm(bf)
# GFtoBF.m:6
    return bf
    
if __name__ == '__main__':
    pass