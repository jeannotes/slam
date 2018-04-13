% Random Walk Simulation
% Copyright(c) 2009-2014, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/07/2011
glvs
ts = 0.1;
t = 1*3600;
len = fix(t/ts);
imuerr = imuerrset(0, 0, 0.01, 10);
imu = imuadderr(zeros(len,6), imuerr, ts);
imu = cumsum(imu, 1);  % accumulate
figure(1);
tt = (1:len)'*ts; 
gg = imuerr.web(1)*sqrt(tt); aa = imuerr.wdb(1)*sqrt(tt);  % reference values
subplot(211), plot(tt,imu(:,1:3)/glv.deg, tt,[gg,-gg]/glv.deg,'m--')
title('Angular Random Walk'); xygo('\it\Delta\theta\rm / \circ');
subplot(212), plot(tt,imu(:,4:6), tt,[aa,-aa],'m--')
title('Velocity Random Walk'); xygo('\it\DeltaV\rm / m/s');
