function [avp, xkpk, ins, kf] = sinsgps(imu, gps, ins, davp, imuerr, lever, r0, fbstr)
global glv
    if ~exist('fbstr', 'var'), fbstr='avp'; end
    [nn, ts, nts] = nnts(4, diff(imu(1:2,end)));
    if size(gps,2)<=5, gpspos_only = 1; end 
    psinstypedef(186);
    kf = [];
    kf.Qt = diag([imuerr.web; imuerr.wdb; zeros(3,1); imuerr.sqg; imuerr.sqa; zeros(3,1)])^2;
    kf.Rk = diag(r0)^2;
    kf.Pxk = diag([davp; imuerr.eb; imuerr.db; lever]*1.0)^2;
    kf.Hk = zeros(length(r0),18);
    kf = kfinit0(kf, nts);
    imugpssyn(imu(:,7), gps(:,end));
    len = length(imu); [avp, xkpk] = prealloc(fix(len/nn), 10, 2*kf.n+1);
    timebar(nn, len, '18-state SINS/GPS simulation.'); ki = 1;
    kfs = kfstatistic(kf);    kfs1 = kfstat([],kf);
    for k=1:nn:len-nn+1
        k1 = k+nn-1; 
        wvm = imu(k:k1,1:6); t = imu(k1,end);
        ins = insupdate(ins, wvm);
        kf.Phikk_1 = kffk(ins);
        kf = kfupdate(kf);
%         kfs = kfstatistic(kfs, kf, 'T');   kfs1 = kfstat(kfs1, kf, 'T');
        [kgps, dt] = imugpssyn(k, k1, 'F');
        if kgps>0
            if gps(kgps,4)<4 && gps(kgps,4)>0   % DOP
                ins = inslever(ins);
                if gpspos_only==1
                    zk = ins.posL-gps(kgps,1:3)'; 
                    kf.Hk = [zeros(3,6), eye(3), zeros(3,6), -ins.MpvCnb];
                else
                    zk = [ins.vnL;ins.posL]-gps(kgps,1:6)';
                    kf.Hk = [zeros(6,3), eye(6), zeros(6,6), [-ins.CW;-ins.MpvCnb]];
                end
                kf = kfupdate(kf, zk, 'M');
                [kf, ins] = kffeedback(kf, ins, 1, fbstr);
    %             kfs = kfstatistic(kfs, kf, 'M');     kfs1 = kfstat(kfs1, kf, 'M');
            end
        end
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]'; xkpk(ki,16:18)=ins.an'; ki = ki+1;
        timebar;
    end
%     kfs = kfstatistic(kfs);    kfs1 = kfstat(kfs1,kf);
    avp(ki:end,:) = []; xkpk(ki:end,:) = [];
    insplot(avp);
    kfplot(xkpk);

