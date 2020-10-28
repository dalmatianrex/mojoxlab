function mq = multiplicationQuaternions(x,y)
fi=x(1)*y(1)-x(2)*y(2)-x(3)*y(3)-x(4)*y(4);
se=x(1)*y(2)+x(2)*y(1)+x(3)*y(4)-x(4)*y(3);
th=x(1)*y(3)-x(2)*y(4)+x(3)*y(1)+x(4)*y(2);
fo=x(1)*y(4)+x(2)*y(3)-x(3)*y(2)+x(4)*y(1);
mq=transpose([fi se th fo]);

mq=mq/norm(mq);
end