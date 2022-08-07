
% Copyright 2015 The MathWorks, Inc.

function [c,ceq] = circlecon(x)
%c = (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;
c = [1,0]*[x(1);x(2)]+3/2;
ceq = [];