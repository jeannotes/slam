#include "stdafx.h"

void main(void) {
//	CMat3 m(1,2,6, 7,2,2, 8,1,4), u, s, v, c;
    CMat3 m(0, -1, 0, 0, 0, 2, 1, 0, 0), u, s, v, c;
    svd3(m, u, s, v);
    c = u * (~v);
    c = u * s * (~v);
    double aa  = 1;
}

void main1(void) {
    FILE *fout, *foutkf;
    fout = fopen("H:\\psins150826\\data\\vc60res.txt", "wt");
    foutkf = fopen("H:\\psins150826\\data\\vc60reskf.txt", "wt");
    CIMUFile imufile("H:\\psins150826\\data\\lasergyro.imu");
    imufile.Skip((int)(40 / imufile.ts + 0.1));
//	imufile.att0 = CVect3(9.005593e-01, 1.676776e-01, -9.059221e+01)*glv.deg;  // true attitude
    CVect3 wm[5], vm[5], wmm(0.0), vmm(0.0);
    int nn = 2;
    CSINS sins(CQuat(imufile.att0), imufile.vn0, imufile.pos0);
    CKalman kf(15, 6);
    kf.SetPk(glv.deg, glv.deg, 10.0 * glv.deg, 1.0, 1.0, 1.0, 10.0 / glv.Re, 10.0 / glv.Re, 10.0,
             0.01 * glv.dph, 0.01 * glv.dph, 0.01 * glv.dph, 1.000 * glv.ug, 1.000 * glv.ug, 100.0 * glv.ug);
    kf.SetQt(0.001 * glv.dpsh, 0.001 * glv.dpsh, 0.001 * glv.dpsh, 1.0 * glv.ugpsHz, 1.0 * glv.ugpsHz, 1.0 * glv.ugpsHz, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 10.0 * glv.ugpsh, 10.0 * glv.ugpsh, 10.0 * glv.ugpsh);
    kf.SetRk(0.1, 0.1, 0.1, 1.0 / glv.Re, 1.0 / glv.Re, 1.0);
    kf.SetHk();
    CVect3 *phi = (CVect3*)&kf.Xk.dd[0], *dvn = (CVect3*)&kf.Xk.dd[3], *dpos = (CVect3*)&kf.Xk.dd[6];
    for (int i = 0; i < 2000 / imufile.ts; i += nn) {
        if (!imufile.Load(wm, vm, nn)) break;
        if (i < 30 / imufile.ts) { // coarse align
            for (int j = 0; j < nn; j++) {
                wmm = wmm + wm[j];
                vmm = vmm + vm[j];
            }
            sins.qnb = CQuat(AlignCoarse(wmm, vmm, imufile.pos0.i));
        } else {
            sins.Update(wm, vm, nn, imufile.ts);
            if (i < 500 / imufile.ts) {	// fine align
                kf.SetFt(sins);
                kf.TimeUpdate(sins.nts);
                if (i % 100 == 0) {
                    CVect3 zkv = sins.vn - CVect3(0.0), zkp = sins.pos - imufile.pos0;
                    kf.SetZk(zkv.i, zkv.j, zkv.k, zkp.i, zkp.j, zkp.k);
                    kf.MeasUpdate();
                    sins.qnb = sins.qnb - *phi;
                    *phi = CVect3(0.0);
                    sins.vn = sins.vn - *dvn;
                    *dvn = CVect3(0.0);
                    sins.pos = sins.pos - *dpos;
                    *dpos = CVect3(0.0);  // height feedback
                    for (int j = 0; j < kf.q; j++)	fprintf(foutkf, "%.8lf ", kf.Xk(j));
                    for (    j = 0; j < kf.q; j++)	fprintf(foutkf, "%.8lf ", sqrt(kf.Pk(j, j)));
                    fprintf(foutkf, "%.8lf \n", imufile.t);
                }
            }
            fprintf(fout, "%.4lf %.4lf %.4lf %lf %lf %lf %.8lf %.8lf %.3lf %.3lf\n",
                    sins.att.i, sins.att.j, sins.att.k, sins.vn.i, sins.vn.j, sins.vn.k,
                    sins.pos.i, sins.pos.j, sins.pos.k, imufile.t);
        }
    }
    fclose(fout);
    fclose(foutkf);
}

/* Use the following command to show the results in Matlab/PSINS Toolbox:
ins = load('vc60res.txt');  xkpk = load('vc60reskf.txt');
insplot(ins); inserrplot(xkpk(:,[1:15,end])); inserrplot(xkpk(:,16:end));
*/
