@function
def e2B(x=None,y=None,*args,**kwargs):
    varargin = e2B.varargin
    nargin = e2B.nargin

    #Floating axis
    c=cross(x,y)
# e2B.m:3
    e2=c / norm(c)
# e2B.m:4
    return e2
    
if __name__ == '__main__':
    pass