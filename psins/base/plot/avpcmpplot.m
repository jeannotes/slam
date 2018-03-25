function avpcmpplot(avp0, varargin)
% AVPs comparison & errors plot.
%
% Prototype: avpcmpplot(avp0, varargin)
% Inputs: avp0 - AVP reference, may be [att], [att,vn], [att,vn,pos]
%                or [vn,pos]
%         varargin - the last input parameter may be comparison type or by
%                    default value
%          
% See also  inserrplot, kfplot, gpsplot, imuplot.

% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 06/10/2013
global glv
    n = size(avp0,2);
    if ischar(varargin{end}),  ptype = varargin{end};  varargin = varargin(1:end-1);
    else
        if n<6,  	ptype = 'a';
        elseif n<9,	ptype = 'av';
        else      	ptype = 'avp';
        end
    end
    %%
    myfigure;
    switch ptype
        case 'a',
            avp = avp0; t = avp(:,end);
            subplot(121), plot(t, avp(:,1:3)/glv.deg), xygo('att');
        case 'av',
            avp = avp0; t = avp(:,end);
            subplot(221), plot(t, avp(:,1:3)/glv.deg), xygo('att');
            subplot(223), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
        case 'avp',
            avp = avp0; t = avp(:,end);
            subplot(321), plot(t, avp(:,1:3)/glv.deg), xygo('att');
            subplot(323), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(325), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
        case 'vp',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
%             subplot(221), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(221), plot(t, avp(:,4:6)); xygo('V');
            subplot(223), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
        case 'v',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
%             subplot(121), plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)]); xygo('V');
            subplot(121), plot(t, avp(:,4:6)); xygo('V');
        case 've',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,4)); xygo('V');
        case 'vn',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,5)); xygo('V');
        case 'vu',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, avp(:,6)); xygo('V');
        case 'p',
            avp = avp0; t = avp(:,end);
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
            if size(avp0,2)<10, avp0 = [zeros(length(avp0),3), avp0]; end
            subplot(121), plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)]); xygo('DP');
    end
    kk = length(varargin);
    str = '-.--: ';
    switch ptype
        case 'a',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(121), hold on, plot(t, avp(:,1:3)/glv.deg, strk, 'LineWidth',2), xygo('att');
                err = avpcmp(avp, avp0, 'mu'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,1:3)/glv.min, strk, 'LineWidth',2); xygo('mu'); mylegend('mux','muy','muz');
            end
         case 'av',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(221), hold on, plot(t, avp(:,1:3)/glv.deg, strk, 'LineWidth',2), xygo('att');
                subplot(223), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'mu'); t = err(:,end);
                subplot(222), hold on, plot(t, err(:,1:3)/glv.min, strk, 'LineWidth',2); xygo('mu'); mylegend('mux','muy','muz');
                subplot(224), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
            end
        case 'avp',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                subplot(321), hold on, plot(t, avp(:,1:3)/glv.deg), xygo('att');
                subplot(323), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(325), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp, avp0, 'mu'); t = err(:,end);
                subplot(322), hold on, plot(t, err(:,1:3)/glv.min, strk, 'LineWidth',2); xygo('mu'); mylegend('mux','muy','muz');
                subplot(324), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
                subplot(326), hold on, plot(t, [err(:,7:8)*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
        case 'vp',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
%                 subplot(221), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(221), hold on, plot(t, avp(:,4:6), strk, 'LineWidth',2); xygo('V');
                subplot(223), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(222), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
                subplot(224), hold on, plot(t, [err(:,7:8)*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
        case 'v',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
%                 subplot(121), hold on, plot(t, [avp(:,4:6),sqrt(avp(:,4).^2+avp(:,5).^2+avp(:,6).^2)], strk, 'LineWidth',2); xygo('V');
                subplot(121), hold on, plot(t, avp(:,4:6), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,4:6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE','dVN','dVU');
            end
        case 've',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,4), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,4), strk, 'LineWidth',2); xygo('dV'); mylegend('dVE');
            end
        case 'vn',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,5), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,5), strk, 'LineWidth',2); xygo('dV'); mylegend('dVN');
            end
        case 'vu',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, avp(:,6), strk, 'LineWidth',2); xygo('V');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, err(:,6), strk, 'LineWidth',2); xygo('dV'); mylegend('dVU');
            end
        case 'p',
            for k=1:kk
                strk = str(k*2-1:k*2);
                avp = varargin{k}; t = avp(:,end);
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                if size(avp,2)<10, avp = [zeros(length(avp),3), avp]; end
                subplot(121), hold on, plot(t, [[avp(:,7)-avp0(1,7),(avp(:,8)-avp0(1,8))*cos(avp0(1,7))]*glv.Re,avp(:,9)-avp0(1,9)], strk, 'LineWidth',2); xygo('DP');
                err = avpcmp(avp, avp0, 'noatt'); t = err(:,end);
                subplot(122), hold on, plot(t, [err(:,7:8)*glv.Re,err(:,9)], strk, 'LineWidth',2); xygo('dP'); mylegend('dlat','dlon','dH');
            end
    end
