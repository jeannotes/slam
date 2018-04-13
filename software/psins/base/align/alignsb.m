function [att, qnb] = alignsb(imu, pos)
% SINS coarse align on static base.
%
% Prototype: [att, qnb] = alignsb(imu, pos)
% Inputs: imu - SIMU data
%         pos - initial position
% Outputs: att, qnb - attitude align results Euler angles & quaternion
%
% See also  dv2atti, alignvn, aligncmps, aligni0, insupdate.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/09/2011
global glv
    wbib = mean(imu(:,1:3),1)'; fbsf = mean(imu(:,4:6),1)';
    lat = asin(wbib'*fbsf/norm(wbib)/norm(fbsf)); % latitude determing via sensor
    if nargin<2     % pos not given
        pos = lat;
    end
    if length(pos)==1
        pos = [pos; 0; 0];
    end
    eth = earth(pos);
    [qnb, att] = dv2atti(eth.gn, eth.wnie, -fbsf, wbib);
    if nargin<2
        resdisp('Coarse align resusts (att,lat_estimated/arcdeg)', ...
            [att; lat]/glv.deg);
    else
        resdisp('Coarse align resusts (att,lat_estimated,lat_real/arcdeg)', ...
            [att; lat; pos(1)]/glv.deg);
    end
