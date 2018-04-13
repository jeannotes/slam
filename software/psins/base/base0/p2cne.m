function Cne = p2cne(pos)
% Convert geographic pos = [lat; lon; *] to transformation matrix Cne 
% from Earth-frame to nav-frame.
%
% Prototype: Cne = p2cne(pos)
% Input: pos - geographic position
% Output: Cne - transformation matrix from Earth-frame to nav-frame
%
% See also  blh2xyz, xyz2blh, a2mat, pp2vn.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 05/05/2010
    slat = sin(pos(1)); clat = cos(pos(1));
    slon = sin(pos(2)); clon = cos(pos(2));
    Cne = [ -slon,       clon,      0
            -slat*clon, -slat*slon, clat
             clat*clon,  clat*slon, slat ];
