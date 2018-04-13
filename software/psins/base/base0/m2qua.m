function qnb = m2qua(Cnb)
% Convert direction cosine matrix(DCM) to attitude quaternion.
%
% Prototype: qnb = m2qua(Cnb)
% Input: Cnb - DCM from body-frame to navigation-frame
% Output: qnb - attitude quaternion
%
% See also  a2mat, a2qua, m2att, q2att, q2mat, attsyn, m2rv.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 21/02/2008, 15/03/2014
    qnb = [ 1.0 + Cnb(1,1) + Cnb(2,2) + Cnb(3,3);
            1.0 + Cnb(1,1) - Cnb(2,2) - Cnb(3,3);
            1.0 - Cnb(1,1) + Cnb(2,2) - Cnb(3,3);
            1.0 - Cnb(1,1) - Cnb(2,2) + Cnb(3,3) ]; 
    s = sign([ 1;
               Cnb(3,2)-Cnb(2,3); 
               Cnb(1,3)-Cnb(3,1); 
               Cnb(2,1)-Cnb(1,2) ]);
    qnb = s.*sqrt(abs(qnb))/2;   % sqrt(.) may decrease accuracy
    
%     qnb = [                       sqrt(abs(1.0 + Cnb(1,1) + Cnb(2,2) + Cnb(3,3)))/2.0;
%         sign(Cnb(3,2)-Cnb(2,3)) * sqrt(abs(1.0 + Cnb(1,1) - Cnb(2,2) - Cnb(3,3)))/2.0;
%         sign(Cnb(1,3)-Cnb(3,1)) * sqrt(abs(1.0 - Cnb(1,1) + Cnb(2,2) - Cnb(3,3)))/2.0;
%         sign(Cnb(2,1)-Cnb(1,2)) * sqrt(abs(1.0 - Cnb(1,1) - Cnb(2,2) + Cnb(3,3)))/2.0 ]; 