@function
def DCM(q=None,*args,**kwargs):
    varargin = DCM.varargin
    nargin = DCM.nargin

    #convert from unit quaternions to direction cosine matrix
    q=q / norm(q)
# DCM.m:3
    M=concat([[q(1) ** 2 + q(2) ** 2 - q(3) ** 2 - q(4) ** 2,dot(2,(dot(q(2),q(3)) - dot(q(1),q(4)))),dot(2,(dot(q(2),q(4)) + dot(q(1),q(3))))],[dot(2,(dot(q(2),q(3)) + dot(q(1),q(4)))),q(1) ** 2 - q(2) ** 2 + q(3) ** 2 - q(4) ** 2,dot(2,(dot(q(3),q(4)) - dot(q(1),q(2))))],[dot(2,(dot(q(2),q(4)) - dot(q(1),q(3)))),dot(2,(dot(q(3),q(4)) + dot(q(1),q(2)))),q(1) ** 2 - q(2) ** 2 - q(3) ** 2 + q(4) ** 2]])
# DCM.m:4
    return M
    
if __name__ == '__main__':
    pass