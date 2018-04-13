function out = delbias(in)
% Delete bias.
%
% Prototype:  out = delbias(in)
% Input: in - input date with bias
% Output: out - output date with no bias
%
% See also  sumn, meann.

% Copyright(c) 2009-2016, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 23/07/2016

    out = in;
    for k=1:size(in,2)
        out(:,k) = in(:,k)-mean(in(:,k));
    end