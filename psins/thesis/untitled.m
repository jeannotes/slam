clc,clear
glvs





% avp0 = avpset([0;0;0], [0;0;0], [34;110;380]);  eth = earth(avp0(7:9));
% imuerr = imuerrset([0;0;0.], [0;0;0], 0.0, 0.0);
% imu = imustatic(avp0, ts, T, imuerr);   % SIMU simulation
% davp0 = avpseterr([0;0;0], [0.0;10.0;0.0], [0;0;0]);
% avp00 = avpadderr(avp0, davp0);
% ins = insinit(avp00, ts);
% len = length(imu);    avp = zeros(fix(len/nn), 10);
% ki = timebar(nn, len, 'Pure inertial navigation processing.');
% for k=1:nn:len-nn+1
% 	k1 = k+nn-1;
% 	wvm = imu(k:k1, 1:6);  t = imu(k1,7);
% 	ins = insupdate(ins, wvm);  ins.vn(3) = 0;  ins.pos(3) = avp0(9);
% 	avp(ki,:) = [ins.avp; t]';
% 	ki = timebar;
% end
% avperr = avpcmp(avp, avp0);
% inserrplot(avperr);