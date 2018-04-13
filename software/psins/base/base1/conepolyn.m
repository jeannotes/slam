function dphim = conepolyn(wm)
% Calculation of noncommutativity error using polynomial compensation 
% method. Ref. Qin Yongyuan 'Inertial Navigation' P314.
%
% Prototype: dphim = conepolyn(wm)
% Input: wm - gyro angular increments
% Output: dphim - noncommutativity error compensation vector
%
% See also  conetwospeed, conedrift, scullpolyn, cnscl.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 09/03/2014
    n = size(wm,1);
    if n==1
        dphim = [0,0,0];
    elseif n==2
        dphim = 2/3*cross(wm(1,:),wm(2,:));
    elseif n==3
        dphim = ...
            33/80*cross(wm(1,:),wm(3,:)) + ...
            57/80*cross(wm(2,:),wm(3,:)-wm(1,:));
    elseif n==4
        dphim = ...
            736/945*(cross(wm(1,:),wm(2,:))+cross(wm(3,:),wm(4,:))) + ...
            334/945*(cross(wm(1,:),wm(3,:))+cross(wm(2,:),wm(4,:))) + ...
            526/945*cross(wm(1,:),wm(4,:)) + ...
            654/945*cross(wm(2,:),wm(3,:));
    else
        error('no suitable compensation in conepolyn');
    end