/* PSINS c++ hearder file PSINS.h */
/*
	By     : Yan Gongmin @ NWPU
	Date   : 2015-02-17, 2015-04-25
	From   : College of Automation, 
	         Northwestern Polytechnical University, 
			 Xi'an 710072, China
*/

#ifndef _PSINS_H
#define _PSINS_H

#include <math.h>
#include <stdio.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <time.h>

/* type re-define */
#ifndef BOOL
typedef int				BOOL;
#endif

/*constant define*/
#ifndef TRUE
#define TRUE	1
#define FALSE	0
#endif

#ifndef NULL
#define NULL	((void *)0)
#endif

#define PI		3.14159265358979

#define EPS		2.22044604925031e-16
#define INF		1.0e100

BOOL assert(BOOL b);
int signE(double val, double eps);
double range(double val, double minVal, double maxVal);
double atan2Ex(double y, double x);
double fastvxv(double *a, double *b, int n);
#define sign(val)		signE(val, EPS)
#define asinEx(x)		asin(range(x, -1.0, 1.0))
#define acosEx(x)		acos(range(x, -1.0, 1.0))

// class define
class CGLV;
class CVect3;	class CMat3;	class CQuat;
class CEarth;	class CIMU;		class CSINS;  class CConfig;
class CMat;		class CKalman; 

// Max Matrix Dimension define
#define MMD	15
#define MMD2	(MMD*MMD)
#define CVect  CMat

// global variables and functions, can not be changed in any way
extern CVect3	O31;
extern CMat3	I33;
extern CGLV	    glv;


class CGLV
{
public:
	double Re, f, g0, wie;
	double e, e2;
	double mg, ug, deg, min, sec, hur, ppm, ppmpsh;
	double dph, dpsh, dphpsh, ugpsh, ugpsHz, mpsh, mpspsh, secpsh;

	CGLV(double Re=6378137.0, double f=(1.0/298.257), double wie0=7.2921151467e-5, double g0=9.7803267714);
};

class CVect3 
{
public:
	double i, j, k;

	CVect3(void) {};
	CVect3(double xx, double yy=0.0, double zz=0.0)  {i=xx, j=yy, k=zz;};
	CVect3(double *pdata)  {i=pdata[0], j=pdata[1], k=pdata[2];};
	CVect3(CMat3 &Cnb); //  {i=asinEx(Cnb.e21), j=atan2Ex(-Cnb.e20, Cnb.e22), k=atan2Ex(-Cnb.e01, Cnb.e11);}; // m2att
	CVect3(CQuat &qnb); //  {*this=CVect3(CMat3(qnb));};  // q2att

	CVect3 operator+(CVect3 &v)  {return CVect3(i+v.i, j+v.j, k+v.k);};
	CVect3 operator-(CVect3 &v)  {return CVect3(i-v.i, j-v.j, k-v.k);};
	CVect3 operator*(CVect3 &v)  {return CVect3(j*v.k-k*v.j, k*v.i-i*v.k, i*v.j-j*v.i);};  // cross
	CVect3 operator*(double f)  {return CVect3(i*f, j*f, k*f);};
	CVect3 operator/(double f)  {return CVect3(i/f, j/f, k/f);};
	friend CVect3 operator*(double f, CVect3 &v)  {return CVect3(v.i*f, v.j*f, v.k*f);};
	friend CVect3 operator-(CVect3 &v)  {return CVect3(-v.i, -v.j, -v.k);};  // minus

	friend double norm(CVect3 &v)  {return sqrt(v.i*v.i + v.j*v.j + v.k*v.k);};  // norm
	friend double dot(CVect3 v1, CVect3 &v2)  {return (v1.i*v2.i + v1.j*v2.j + v1.k*v2.k);};
	friend CQuat rv2q(CVect3 &v);  // rotation vector to quaternion
	friend CMat3 askew(CVect3 &v); //  {return CMat3(0,-v.k,v.j, v.k,0,-v.i, -v.j,v.i,0);};
};

class CQuat
{
public:
	double q0, q1, q2, q3;

	CQuat(void) {};
	CQuat(double qq0, double qq1=0.0, double qq2=0.0, double qq3=0.0)  {q0=qq0, q1=qq1, q2=qq2, q3=qq3;};
	CQuat(CVect3 &att); //  {*this=CQuat(CMat3(att));};  // v2qua
	CQuat(CMat3 &Cnb);  // m2qua

	CQuat operator+(CVect3 &v)  {return rv2q(CVect3(0.0)-v)*(*this);};  // qaddphi
	CQuat operator-(CVect3 &v)  {return rv2q(v)*(*this);};              // qdelphi
	CVect3 operator-(CQuat &quat);   // qq2phi
	CQuat operator*(CQuat &q);
	friend CQuat operator~(CQuat &q)  {return CQuat(q.q0,-q.q1,-q.q2,-q.q3);}; // qconj
	CVect3 operator*(CVect3 &v);     // qmulv
	void normlize(CQuat *q);
	friend CVect3 q2rv(CQuat &q);
	friend CVect3 pp2vn(CVect3 &pos1, CVect3 &pos0, double ts=1.0, CEarth *pEth=NULL);
};

class CMat3 
{
public:
	double e00, e01, e02, e10, e11, e12, e20, e21, e22;

	CMat3(void) {};
	CMat3(double xx, double xy, double xz, 
		  double yx, double yy, double yz,
		  double zx, double zy, double zz )  {e00=xx,e01=xy,e02=xz; e10=yx,e11=yy,e12=yz; e20=zx,e21=zy,e22=zz;};
	CMat3(CVect3 &att);  // v2mat
	CMat3(CQuat &qnb);   // q2mat

	friend CMat3 operator~(CMat3 &m)  {return CMat3(m.e00,m.e10,m.e20, m.e01,m.e11,m.e21, m.e02,m.e12,m.e22);};
	CMat3 operator*(CMat3 &m);
	CMat3 operator+(CMat3 &m);
	CMat3 operator-(CMat3 &m);
	CMat3 operator*(double f)  {return CMat3(e00*f,e01*f,e02*f, e10*f,e11*f,e12*f, e21*f,e20*f,e22*f);};
	friend CMat3 operator*(double f, CMat3 &m)  {return CMat3(m.e00*f,m.e01*f,m.e02*f, m.e10*f,m.e11*f,m.e12*f, m.e20*f,m.e21*f,m.e22*f);};
	CVect3 operator*(CVect3 &v)  {return CVect3(e00*v.i+e01*v.j+e02*v.k,e10*v.i+e11*v.j+e12*v.k,e20*v.i+e21*v.j+e22*v.k);};
	friend double det(CMat3 &m);
	friend CMat3 inv(CMat3 &m);
	friend int svd3(CMat3 &A, CMat3 &u, CMat3 &s, CMat3 &v);
	friend CVect3 diag(CMat3 &m) {return CVect3(m.e00, m.e11, m.e22);};
	friend CMat3 diag(CVect3 &v) {return CMat3(v.i,0,0, 0,v.j,0, 0,0,v.k);};
};

class CMat
{
public:
	int row, clm, rc;
	double dd[MMD2];

	CMat(void) {};
	CMat(int row0, int clm0=1)  { row=row0; clm=clm0; rc=row*clm; };
	CMat(int row0, int clm0, double f)  { row=row0; clm=clm0; rc=row*clm; for(int i=0;i<rc;i++) dd[i]=f; };

	friend CMat operator~(CMat &m);
	CMat operator*(CMat &m);
	CMat operator+(CMat &m);
	CMat operator-(CMat &m);
	CMat operator*(double f);
	double& operator()(int r, int c=0)  { return this->dd[r*this->clm+c]; };
	CVect operator()(char RorC, int i);
};

class CKalman
{
public:
	int q, r;
	CVect Xk, Zk;
	CMat Ft, Pk, Qt, Hk, Rk;

	CKalman(int q0, int r0);
	void SetPk(double f, ...);
	void SetQt(double f, ...);
	void SetRk(double f, ...);
	void SetZk(double f, ...);
	void SetFt(CSINS &sins);
	void SetHk(void);
	void TimeUpdate(double ts);
	void MeasUpdate(double fading=1.0);
};

class CEarth
{
public:
	double a, b;
	double f, e, e2, ep, ep2;
	double wie;

	double lti, hgt, sl, sl2, sl4, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;;
	CVect3 wnie, wnen, wnin, gn, gcc;

	CEarth(double a0=glv.Re, double f0=glv.f, double g0=glv.g0);

	void Update(CVect3 &pos, CVect3 &vn=CVect3(0.0));
	CVect3 vn2dpos(CVect3 &vn, double ts=1.0)  {return CVect3(vn.j*f_RMh, vn.i*f_clRNh, vn.k)*ts;};
};

class CIMU
{
public:
	int nSamples, prefirst;
	CVect3 phim, dvbm, wm_1, vm_1;

	CIMU(void) {prefirst=1;};
	void Update(CVect3 *wm, CVect3 *vm, int nSamples);
};

class CSINS
{
public:
	double nts;
	CEarth eth;
	CIMU imu;
	CQuat qnb;
	CMat3 Cnb, Cnb0, Kg, Ka;
	CVect3 wib, fb, fn, an, web, wnb, att, vn, pos, eb, db;

	CSINS(void) {};
	CSINS(CQuat &qnb0, CVect3 &vn0, CVect3 &pos0);

	void Update(CVect3 *wm, CVect3 *vm, int nSamples, double ts);
	void etm(CMat3 &Maa, CMat3 &Mav, CMat3 &Map, 
		CMat3 &Mva, CMat3 &Mvv, CMat3 &Mvp, CMat3 &Mpv, CMat3 &Mpp);
};

class CIMUFile
{
	FILE *f;
public:
	CVect3 att0, vn0, pos0, gf, af;
	double t0, ts, t, g0;

	CIMUFile(char *fname);
	int Skip(int n);
	int Load(CVect3 *wm, CVect3 *vm, int n);
	~CIMUFile() { if(f) fclose(f); }
};

CMat3 AlignCoarse(CVect3 wmm, CVect3 vmm, double latitude);

#endif