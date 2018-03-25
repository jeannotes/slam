function [qnb, att, Cnb] = dv2atti(vn1, vn2, vb1, vb2)
% Using double-measurement vectors to determine attitude.
%
% Prototype: [qnb, att, Cnb] = dv2atti(vn1, vn2, vb1, vb2)
% Inputs: vn1,vn2 - two reference vectors in nav-frame
%         vb1,vb2 - two measurement vectors in body-frame
% Outputs: qnb, att, Cnb - the same attitude representations in 
%               quaternion, Euler angles & DCM form
% Example:
%    [qnb, att, Cnb] = dv2atti(gn, wnie, -fb, wb);
%       where gn is gravity vector and fb is acc specific force output,
%       while wnie is the Earth angular velocity and wb is gyro output.
%
% See also  sv2atti, mv2atti, alignsb.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/09/2012
    vb1 = norm(vn1)/norm(vb1)*vb1;
    vb2 = norm(vn2)/norm(vb2)*vb2;
    vntmp = cross(vn1,vn2);
    vbtmp = cross(vb1,vb2);
    Cnb = [vn1'; vntmp'; cross(vntmp,vn1)']^-1 * [vb1'; vbtmp'; cross(vbtmp,vb1)'];
    for k=1:5   % normalization
        Cnb = 0.5 * (Cnb + (Cnb')^-1);
    end
    qnb = m2qua(Cnb); qnb = qnb/norm(qnb);
    [qnb, att, Cnb] = attsyn(qnb);