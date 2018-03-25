function vn = pp2vn(pos0, pos1, ts)
% Use differential positions to get average velocity.
% Denoted as vn = (pos1-pos0)/ts.
%
% Prototype: vn = pp2vn(pos0, pos1, ts)
% Inputs: pos0, pos1 - geographic position at time t0 and t1
%         ts - time interval between t0 and t1, i.e. ts = t1-t0
% Output: vn - average velocity
%
% See also  vn2dpos, p2cne.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 03/10/2013
global glv
    if nargin<3
        ts = 1;
    end
    sl=sin(pos0(1)); cl=cos(pos0(1)); sl2=sl*sl;
    sq = 1-glv.e2*sl2; sq2 = sqrt(sq);
    RMh = glv.Re*(1-glv.e2)/sq/sq2+pos0(3);
    RNh = glv.Re/sq2+pos0(3);    clRNh = cl*RNh;
    vn = pos1 - pos0;
    vn = [vn(2)*clRNh; vn(1)*RMh; vn(3)] / ts;