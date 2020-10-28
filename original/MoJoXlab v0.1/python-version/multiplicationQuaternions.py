@function
def multiplicationQuaternions(x=None, y=None, *args, **kwargs):
    varargin = multiplicationQuaternions.varargin
    nargin = multiplicationQuaternions.nargin

    fi = dot(x(1), y(1)) - dot(x(2), y(2)) - dot(x(3), y(3)) - dot(x(4), y(4))
# multiplicationQuaternions.m:2
    se = dot(x(1), y(2)) + dot(x(2), y(1)) + dot(x(3), y(4)) - dot(x(4), y(3))
# multiplicationQuaternions.m:3
    th = dot(x(1), y(3)) - dot(x(2), y(4)) + dot(x(3), y(1)) + dot(x(4), y(2))
# multiplicationQuaternions.m:4
    fo = dot(x(1), y(4)) + dot(x(2), y(3)) - dot(x(3), y(2)) + dot(x(4), y(1))
# multiplicationQuaternions.m:5
    mq = transpose(concat([fi, se, th, fo]))
# multiplicationQuaternions.m:6
    mq = mq / norm(mq)
# multiplicationQuaternions.m:8
    return mq


if __name__ == '__main__':
    pass
