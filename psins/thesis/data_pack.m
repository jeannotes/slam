clear,clc,close all

data = load('first_data.mat');
glvs
psinstypedef(151);
[nn, ts, nts] = nnts(1, 0.005);
acce = data.Accelerometer
acce=(double(acce.signals.values)-2048)/52.244897959183675;
gyro = double(data.Gyroscope.signals.values)*0.0061;
imu = [gyro*ts   acce*ts  data.Accelerometer.time];
imu = imu(2:end,:);

%for initial alignment
imu_alignment = imu(1:100,:);
imu_alignment(:,1:6) = repmat(imu(1,1:6),100,1);

[attsb, qnb] = alignsb(imu_alignment, glv.pos0);
avp0 = avpset(attsb, [0;0;0], glv.pos0);




%for kalman filter
imuerr = imuerrset(0.01, 50, 0.0005, 3);
imu = imuadderr(imu, imuerr);
davp0 = avpseterr([30;-30;20], 0.1, [1;1;3]);

% pure navigation
avp_pure = inspure(imu, avp0, avp0(9));

ins = insinit(avpadderr(avp0,davp0), ts);  ins.nts = nts;
% KF filter
kf = kfinit(ins, davp0, imuerr);
len = length(imu); [avp, xkpk] = prealloc(fix(len*ts), 10, 2*kf.n+1);
% timebar(nn, len, '15-state SINS/GPS Simulation.'); 
ki = 1;
tbstep = floor(len/nn/100); tbi = timebar(1, 99, 'SINS/GPS Simulation.');

for k=1:nn:len-nn+1
    k1 = k+nn-1;  
    wvm = imu(k:k1,1:6);  t = imu(k1,end);
    ins = insupdate(ins, wvm);
    kf.Phikk_1 = kffk(ins);
    kf = kfupdate(kf);
    if 1
%         posGPS = trj.avp(k1,7:9)' + davp0(7:9).*randn(3,1);  % GPS pos simulation with some white noise
        posGPS = glv.pos0(3);
        kf = kfupdate(kf, ins.pos(3)-posGPS, 'B');
        [kf, ins] = kffeedback(kf, ins, 1, 'vp');
        avp(ki,:) = [ins.avp', t];
        xkpk(ki,:) = [kf.xk; diag(kf.Pxk); t]';  ki = ki+1;
    end
%     timebar;
    if mod(tbi,tbstep)==0, timebar; end;  tbi = tbi+1;
end
% profile viewer
% show results
%avperr = avpcmp(avp, trj.avp);
insplot(avp);
