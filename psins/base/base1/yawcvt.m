function yaw = yawcvt(yaw, cvstr)
% Euler yaw angles convertion to designated convension.
%
% Prototype: yaw = yawcvt(yaw, dirstr)
% Inputs: yaw - input yaw angles, in rad
%         dirstr - convention descript string
% Output: yaw - output yaw angles in new convention
%
% See also  m2att, q2att.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 30/05/2014
    switch cvstr
        case 'c360cc180',  % clockwise 0->360deg to counter-clockwise -180->180deg
            idx = yaw>pi;
            yaw(idx) = 2*pi-yaw(idx);
            yaw(~idx) = -yaw(~idx);
        case 'cc360cc180',  % clockwise 0->360deg to counter-clockwise -180->180deg
            yaw = -yawcvt(yaw, 'c360cc180');
        case 'cc180c360',  % counter-clockwise -180->180deg to clockwise 0->360deg
            idx = yaw>0;
            yaw(idx) = 2*pi-yaw(idx);
            yaw(~idx) = -yaw(~idx);
    end
