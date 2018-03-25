#include "stdafx.h"
#include "PSINS.h"

CVect3 O31(0,0,0);
CMat3 I33(1,0,0, 0,1,0, 0,0,1);
CGLV glv;

/***************************  class CGLV  *********************************/
CGLV::CGLV(double Re, double f, double wie0, double g0)
{
	this->Re = Re; this->f = f; this->wie = wie0; this->g0 = g0;
	e = sqrt(2*f-f*f); e2 = e*e;
    mg = 1.0e-3*g0;
    ug = 1.0e-6*glv.g0;
    deg = PI/180.0;
    min = deg/60.0;
    sec = min/60.0;
    ppm = 1.0e-6;
    hur = 3600.0;
    dph = deg/hur;
    dpsh = deg/sqrt(hur);
    dphpsh = dph/sqrt(hur);
    ugpsHz = ug/sqrt(1.0);
    ugpsh = ug/sqrt(hur);
    mpsh = 1/sqrt(hur); 
    mpspsh = 1/1/sqrt(hur); 
    ppmpsh = ppm/sqrt(hur);
    secpsh = sec/sqrt(hur);
}

/***************************  class CVect3  *********************************/
CVect3::CVect3(CMat3 &Cnb)
{
	i=asinEx(Cnb.e21), j=atan2Ex(-Cnb.e20, Cnb.e22), k=atan2Ex(-Cnb.e01, Cnb.e11);
}

CVect3::CVect3(CQuat &qnb)
{
	*this=CVect3(CMat3(qnb));
}

CQuat rv2q(CVect3 &v)
{
#define F1	(   2 * 1)		// define: Fk=2^k*k! 
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F3*2 * 5)
	double n2 = v.i*v.i+v.j*v.j+v.k*v.k, c, f;
	if(n2<(PI/180.0*PI/180.0))	// 0.017^2 
	{
		double n4=n2*n2;
		c = 1.0 - n2*(1.0/F2) + n4*(1.0/F4);
		f = 0.5 - n2*(1.0/F3) + n4*(1.0/F5);
	}
	else
	{
		double n_2 = sqrt(n2)/2.0;
		c = cos(n_2);
		f = sin(n_2)/n_2*0.5;
	}
	return CQuat(c, f*v.i, f*v.j, f*v.k);
}

CVect3 pp2vn(CVect3 &pos1, CVect3 &pos0, double ts, CEarth *pEth)
{
	double sl, cl, sl2, sq, sq2, RMh, RNh, clRNh;
	if(pEth)
	{
		RMh = pEth->RMh; clRNh = pEth->clRNh;
	}
	else
	{
		sl=sin(pos0.i); cl=cos(pos0.i); sl2=sl*sl;
		sq = 1-glv.e2*sl2; sq2 = sqrt(sq);
		RMh = glv.Re*(1-glv.e2)/sq/sq2+pos0.k;
		RNh = glv.Re/sq2+pos0.k;    clRNh = cl*RNh;
	}
    CVect3 vn = pos1 - pos0;
    return CVect3(vn.j*clRNh/ts, vn.i*RMh/ts, vn.k/ts);
}

/***************************  class CQuat  *********************************/
CQuat::CQuat(CVect3 &att)
{
	*this=CQuat(CMat3(att));
}

CQuat::CQuat(CMat3 &Cnb)
{
	double tmp;

	tmp = 1.0 + Cnb.e00 - Cnb.e11 - Cnb.e22;
	q1 = sqrt(fabs(tmp))/2.0;
	tmp = 1.0 - Cnb.e00 + Cnb.e11 - Cnb.e22;
	q2 = sqrt(fabs(tmp))/2.0;
	tmp = 1.0 - Cnb.e00 - Cnb.e11 + Cnb.e22;
	q3 = sqrt(fabs(tmp))/2.0;
	tmp = 1.0 - q1*q1 - q2*q2 - q3*q3;
	q0 = sqrt(fabs(tmp));

	if(Cnb.e21 - Cnb.e12 < 0)	/* sign decision */
	{
		q1 = -q1;
	}
	if(Cnb.e02 - Cnb.e20 < 0)
	{
		q2 = -q2;
	}
	if(Cnb.e10 - Cnb.e01 < 0)
	{
		q3 = -q3;
	}

	double nq = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 /= nq; q1 /= nq; q2 /= nq; q3 /= nq; 
}

CVect3 CQuat::operator-(CQuat &quat)
{
	CQuat dq;
	
	dq = quat*~(*this);
	if(dq.q0<0)
	{
		dq.q0=-dq.q0, dq.q1=-dq.q1, dq.q2=-dq.q2, dq.q3=-dq.q3;
	}
	double n2 = acos(dq.q0), f;
	if( sign(n2)!=0 )
	{
		f = 2.0/(sin(n2)/n2);
	}
	else
	{
		f = 2.0;
	}
	return CVect3(dq.q1,dq.q2,dq.q3)*f;
}

CQuat CQuat::operator*(CQuat &quat)
{
	CQuat qtmp;
	qtmp.q0 = q0*quat.q0 - q1*quat.q1 - q2*quat.q2 - q3*quat.q3;
	qtmp.q1 = q0*quat.q1 + q1*quat.q0 + q2*quat.q3 - q3*quat.q2;
	qtmp.q2 = q0*quat.q2 + q2*quat.q0 + q3*quat.q1 - q1*quat.q3;
	qtmp.q3 = q0*quat.q3 + q3*quat.q0 + q1*quat.q2 - q2*quat.q1;
	return qtmp;
}

CVect3 CQuat::operator*(CVect3 &v)
{
	CQuat qtmp;
	CVect3 vtmp;
	qtmp.q0 =         - q1*v.i - q2*v.j - q3*v.k;
	qtmp.q1 = q0*v.i           + q2*v.k - q3*v.j;
	qtmp.q2 = q0*v.j           + q3*v.i - q1*v.k;
	qtmp.q3 = q0*v.k           + q1*v.j - q2*v.i;
	vtmp.i = -qtmp.q0*q1 + qtmp.q1*q0 - qtmp.q2*q3 + qtmp.q3*q2;
	vtmp.j = -qtmp.q0*q2 + qtmp.q2*q0 - qtmp.q3*q1 + qtmp.q1*q3;
	vtmp.k = -qtmp.q0*q3 + qtmp.q3*q0 - qtmp.q1*q2 + qtmp.q2*q1;
	return vtmp;
}

void normlize(CQuat *q)
{
	double nq=sqrt(q->q0*q->q0+q->q1*q->q1+q->q2*q->q2+q->q3*q->q3);
	q->q0 /= nq, q->q1 /= nq, q->q2 /= nq, q->q3 /= nq;
}

CVect3 q2rv(CQuat &q)
{
	CQuat dq;
	dq = q;
	if(dq.q0<0)  { dq.q0=-dq.q0, dq.q1=-dq.q1, dq.q2=-dq.q2, dq.q3=-dq.q3; }
	if(dq.q0>1.0) dq.q0=1.0;
	double n2 = acos(dq.q0), f;
	if(n2>1.0e-20)
		f = 2.0/(sin(n2)/n2);
	else
		f = 2.0;
	return CVect3(dq.q1,dq.q2,dq.q3)*f;
}

/***************************  class CMat3  *********************************/
CMat3::CMat3(CVect3 &att)
{
	double
		si = sin(att.i), ci = cos(att.i),
		sj = sin(att.j), cj = cos(att.j),
		sk = sin(att.k), ck = cos(att.k);
	e00 =  cj*ck - si*sj*sk;	e01 =  -ci*sk;	e02 = sj*ck + si*cj*sk;
	e10 =  cj*sk + si*sj*ck;	e11 =  ci*ck;	e12 = sj*sk - si*cj*ck;
	e20 = -ci*sj;				e21 =  si;		e22 = ci*cj;
}

CMat3::CMat3(CQuat &qnb)
{
	double 
		q11 = qnb.q0*qnb.q0, q12 = qnb.q0*qnb.q1, q13 = qnb.q0*qnb.q2, q14 = qnb.q0*qnb.q3, 
		q22 = qnb.q1*qnb.q1, q23 = qnb.q1*qnb.q2, q24 = qnb.q1*qnb.q3,     
		q33 = qnb.q2*qnb.q2, q34 = qnb.q2*qnb.q3,  
		q44 = qnb.q3*qnb.q3;
    e00 = q11+q22-q33-q44,  e01 = 2*(q23-q14),     e02 = 2*(q24+q13),
	e10 = 2*(q23+q14),      e11 = q11-q22+q33-q44, e12 = 2*(q34-q12),
	e20 = 2*(q24-q13),      e21 = 2*(q34+q12),     e22 = q11-q22-q33+q44 ;
}

CMat3 CMat3::operator*(CMat3 &mat)
{
	CMat3 mtmp;
	mtmp.e00 = e00*mat.e00 + e01*mat.e10 + e02*mat.e20;
	mtmp.e01 = e00*mat.e01 + e01*mat.e11 + e02*mat.e21;
	mtmp.e02 = e00*mat.e02 + e01*mat.e12 + e02*mat.e22;
	mtmp.e10 = e10*mat.e00 + e11*mat.e10 + e12*mat.e20;
	mtmp.e11 = e10*mat.e01 + e11*mat.e11 + e12*mat.e21;
	mtmp.e12 = e10*mat.e02 + e11*mat.e12 + e12*mat.e22;
	mtmp.e20 = e20*mat.e00 + e21*mat.e10 + e22*mat.e20;
	mtmp.e21 = e20*mat.e01 + e21*mat.e11 + e22*mat.e21;
	mtmp.e22 = e20*mat.e02 + e21*mat.e12 + e22*mat.e22;
	return mtmp;
}

CMat3 CMat3::operator+(CMat3 &mat)
{
	CMat3 mtmp;
	mtmp.e00 = e00 + mat.e00;  mtmp.e01 = e01 + mat.e01;  mtmp.e02 = e02 + mat.e02;  
	mtmp.e10 = e10 + mat.e10;  mtmp.e11 = e11 + mat.e11;  mtmp.e12 = e12 + mat.e12;  
	mtmp.e20 = e20 + mat.e20;  mtmp.e21 = e21 + mat.e21;  mtmp.e22 = e22 + mat.e22;  
	return mtmp;
}

CMat3 CMat3::operator-(CMat3 &mat)
{
	CMat3 mtmp;
	mtmp.e00 = e00 - mat.e00;  mtmp.e01 = e01 - mat.e01;  mtmp.e02 = e02 - mat.e02;  
	mtmp.e10 = e10 - mat.e10;  mtmp.e11 = e11 - mat.e11;  mtmp.e12 = e12 - mat.e12;  
	mtmp.e20 = e20 - mat.e20;  mtmp.e21 = e21 - mat.e21;  mtmp.e22 = e22 - mat.e22;  
	return mtmp;
}

CMat3 askew(CVect3 &v)
{
	return CMat3(0,  -v.k, v.j, 
				 v.k, 0,  -v.i,
				-v.j, v.i, 0);
}

double det(CMat3 &m)
{
	return m.e00*(m.e11*m.e22-m.e12*m.e21) - m.e01*(m.e10*m.e22-m.e12*m.e20) + m.e02*(m.e10*m.e21-m.e11*m.e20);
}

CMat3 inv(CMat3 &m)
{
	double nm;
	nm = m.e00*(m.e11*m.e22-m.e12*m.e21) - m.e01*(m.e10*m.e22-m.e12*m.e20) + m.e02*(m.e10*m.e21-m.e11*m.e20);
	CMat3 mtmp;
	mtmp.e00 =  (m.e11*m.e22-m.e12*m.e21)/nm;
	mtmp.e10 = -(m.e10*m.e22-m.e12*m.e20)/nm;
	mtmp.e20 =  (m.e10*m.e21-m.e11*m.e20)/nm;
	mtmp.e01 = -(m.e01*m.e22-m.e02*m.e21)/nm;
	mtmp.e11 =  (m.e00*m.e22-m.e02*m.e20)/nm;
	mtmp.e21 = -(m.e00*m.e21-m.e01*m.e20)/nm;
	mtmp.e02 =  (m.e01*m.e12-m.e02*m.e11)/nm;
	mtmp.e12 = -(m.e00*m.e12-m.e02*m.e10)/nm;
	mtmp.e22 =  (m.e00*m.e11-m.e01*m.e10)/nm;
	return mtmp;
}

int svd3(CMat3 &A, CMat3 &u, CMat3 &s, CMat3 &v)
{
	CMat3 B = (~A)*A;
    double a = -(B.e00+B.e11+B.e22),
		b = B.e00*B.e11+B.e00*B.e22+B.e11*B.e22-B.e12*B.e12-B.e01*B.e01-B.e02*B.e02,
		c = -(B.e00*B.e11*B.e22-B.e00*B.e12*B.e12-B.e01*B.e01*B.e22+2*B.e01*B.e02*B.e12-B.e02*B.e02*B.e11),
	    p = b-a*a/3, q = (2*a*a*a-9*a*b+27*c)/27,
	    delta = q*q/4+p*p*p/27, x1, x2, x3, tmp;
	int rootstate;
	if(delta<-100*EPS)
	{
        double T = -q*sqrt(-27*p)/(2*p*p), theta = acosEx(T), x0 = 2*sqrt(-p/3);
        x1 = x0*cos(theta/3); x2 = x0*cos(theta/3-PI*2.0/3); x3 = x0*cos(theta/3+PI*2.0/3);
        rootstate = 3;
	}
	else if(delta<100*EPS)
	{
		x1 = q>=0.0 ? -pow(4*q,1.0/3) : pow(-4*q,1.0/3);
        x2 = -x1/2; x3 = x2;
        rootstate = 2;
        if(p>-100*EPS) rootstate = 1;
	}
	else
		return 0;  // complex roots, reject
	x1 -= a/3, x2 -= a/3, x3 -= a/3;
	if(x1<x2) tmp = x1, x1 = x2, x2 = tmp;        // sort x1>=x2>=x3
	if(x1<x3) tmp = x1, x1 = x3, x3 = tmp;
	if(x2<x3) tmp = x2, x2 = x3, x3 = tmp;

	CVect3 v1, v2, v3, vi, vt1, vt2, vt3;
	CMat3 C;
    if(rootstate==1)
	{
        v1 = CVect3(1,0,0), v2 = CVect3(0,1,0), v3 = CVect3(0,0,1);
	}
    else if(rootstate==2)
	{
        C = x2*I33 - B;
        double m = 0.0, Cij, *pCm=&C.e00;
		int i, j, I=0, J=0;
        for(i=0; i<3; i++)
		{
            for(j=0; j<3; j++)
			{
                Cij = fabs(*pCm++);
                if(m<Cij) { I=i; J=j; m=Cij; }
            }
        }
		pCm=&C.e00;
        vi = CVect3(pCm[3*I+0], pCm[3*I+1], pCm[3*I+2]);
        v2 = J<2 ? CVect3(vi.j, -vi.i, 0) : CVect3(0, -vi.k, vi.j);
        if(sign(x1-x2)==0)	{ v1 = vi*v2, v3 = v1*v2; }
        else				{ v3 = vi*v2, v1 = v3*v2; }
	}
    else
	{
		double nvt1, nvt2, nvt3;
        C = x1*I33 - B;
        vt1 = *((CVect3*)&C.e00) * *((CVect3*)&C.e10);  nvt1 = norm(vt1); 
        vt2 = *((CVect3*)&C.e10) * *((CVect3*)&C.e20);  nvt2 = norm(vt2);
        vt3 = *((CVect3*)&C.e20) * *((CVect3*)&C.e00);  nvt3 = norm(vt3);
        if(nvt1>nvt2 && nvt1>nvt3) v1 = vt1;
        else if(nvt2>nvt1 && nvt2>nvt3) v1 = vt2;
        else v1 = vt3;
        C = x2*I33 - B;
        vt1 = *((CVect3*)&C.e00) * *((CVect3*)&C.e10);  nvt1 = norm(vt1);
        vt2 = *((CVect3*)&C.e10) * *((CVect3*)&C.e20);  nvt2 = norm(vt2);
        vt3 = *((CVect3*)&C.e20) * *((CVect3*)&C.e00);  nvt3 = norm(vt3);
        if(nvt1>nvt2 && nvt1>nvt3) v2 = vt1;
        else if(nvt2>nvt1 && nvt2>nvt3) v2 = vt2;
        else v2= vt3;
        v3 = v1*v2;
    }

	double nv1=norm(v1), nv2=norm(v2), nv3=norm(v3);
    v = CMat3(	v1.i/nv1, v2.i/nv2, v3.i/nv3,
				v1.j/nv1, v2.j/nv2, v3.j/nv3,
				v1.k/nv1, v2.k/nv2, v3.k/nv3 );
    s = diag(CVect3(sqrt(x1),sqrt(x2),sqrt(x3)));
    u = A*v*CMat3(	sign(s.e00)==0?0:1/s.e00,0,0, 
					0,sign(s.e11)==0?0:1/s.e11,0, 
					0,0,sign(s.e22)==0?0:1/s.e22 );
	return sign(det(A));
}

/***************************  class CMat  *********************************/
/*CMat CMat::operator*(CMat &m0)
{
	assert(this->clm==m0.row);
	CMat mtmp(this->row,m0.clm), mt=(~m0);
	int m=this->row, k=this->clm, n=m0.clm;
	double *p=mtmp.dd, *p1=this->dd, *p2=mt.dd;
	for(int i=0; i<m; i++,p1+=clm)
	{
		p2=mt.dd;
		for(int j=0; j<n; j++,p2+=clm)
		{
			*p++ = fastvxv(p1, p2, k);
		}
	}
	return mtmp;
}*/

CMat CMat::operator*(CMat &m0)
{
	assert(this->clm==m0.row);
	CMat mtmp(this->row,m0.clm);
	int m=this->row, k=this->clm, n=m0.clm;
	double *p=mtmp.dd, *p1i=this->dd, *p2=m0.dd;
	for(int i=0; i<m; i++,p1i+=k)
	{
		for(int j=0; j<n; j++)
		{
//			double f=0.0, *p1is=p1i, *p2sj=&p2[j];
//			for(int s=0; s<k; s++,p1is++,p2sj+=n)	f += (*p1is) * (*p2sj);
			double f=0.0, *p1is=p1i, *p1isEnd=&p1i[k], *p2sj=&p2[j];
			for(; p1is<p1isEnd; p1is++,p2sj+=n)	f += (*p1is) * (*p2sj);
			*p++ = f;
		}
	}
	return mtmp;
}

CMat CMat::operator+(CMat &m0)
{
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc], *p1=this->dd, *p2=m0.dd;
	while(p<pEnd)
	{ *p++ = (*p1++) + (*p2++); } 
	return mtmp;
}

CMat CMat::operator-(CMat &m0)
{
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc], *p1=this->dd, *p2=m0.dd;
	while(p<pEnd)
	{ *p++ = (*p1++) - (*p2++); } 
	return mtmp;
}

CMat CMat::operator*(double f)
{
	CMat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc], *p1=this->dd;
	while(p<pEnd)
	{ *p++ = (*p1++) * f; } 
	return mtmp;
}

CMat operator~(CMat &m0)
{
	CMat mtmp(m0.clm,m0.row);
	double *pm=m0.dd;
	for(int i=0; i<m0.row; i++)
	{ for(int j=i; j<m0.rc; j+=m0.row) mtmp.dd[j] = *pm++; }
	return mtmp;
}

CVect CMat::operator()(char RorC, int i)
{
	CVect v(1,1);
	if(RorC=='R')
	{ v.row=1; v.clm=v.rc=clm;  for(double *p=v.dd,*p1=&dd[i*clm],*pEnd=p1+clm; p1<pEnd; p++,p1++) *p = *p1; }
	else if(RorC=='C')
	{ v.row=v.rc=row; v.clm=1;	for(double *p=v.dd,*p1=&dd[i],*pEnd=&dd[rc]; p1<pEnd; p++,p1+=clm) *p = *p1; }
	return v;
}

/***************************  class CKalman  *********************************/
CKalman::CKalman(int q0, int r0)
{
	assert(q0<=MMD&&r0<=MMD);
	q = q0; r = r0;
	Ft = Qt = Pk = CMat(q,q,0.0);
	Hk = CMat(r,q,0.0);
	Rk = CMat(r,r,0.0);
	Xk = CVect(q,1,0.0);
	Zk = CVect(r,1,0.0);
}

void CKalman::TimeUpdate(double ts)
{
	CMat Fk=Ft*ts;
	for(double *p=Fk.dd, *pEnd=&Fk.dd[Fk.rc]; p<pEnd; p+=Fk.row+1)	{ *p += 1.0; }  // Fk = I+Ft*ts
	Xk = Fk * Xk;
	Pk = Fk*Pk*(~Fk) + Qt*ts;
}

void CKalman::MeasUpdate(double fading)
{
	CVect Pxz, Kk, Hi;
	for(int i=0; i<r; i++)
	{
		Hi = Hk('R',i);
		Pxz = Pk*(~Hi);
		double Pzz = (Hi*Pxz)(0) + Rk(i,i);
		Kk = Pxz*(1.0/Pzz);
		Xk = Xk + Kk*(Zk(i)-(Hi*Xk)(0));
		Pk = Pk - Kk*(~Pxz);
	}
	if(fading>1.0) Pk = Pk*fading;
}

void CKalman::SetPk(double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<q; i++)
	{ Pk.dd[i*q+i] = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CKalman::SetQt(double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<q; i++)
	{ Qt.dd[i*q+i] = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CKalman::SetRk(double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<r; i++)
	{ Rk.dd[i*r+i] = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void CKalman::SetZk(double f, ...)
{
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<r; i++)
	{ Zk.dd[i] = f;  f = va_arg(vl, double); }
	va_end(vl);
}

void CKalman::SetFt(CSINS &sins)
{
	CMat3 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp, Cnb;
	sins.etm(Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp);
	Cnb = sins.Cnb;
//	Ft = [ Maa    Mav    Map    -ins.Cnb  O33 
//         Mva    Mvv    Mvp     O33      ins.Cnb 
//         O33    Mpv    Mpp     O33      O33
//         zeros(6,9)  diag(-1./[ins.tauG;ins.tauA]) ];
	// phi
	                 ; Ft(0,1) = Maa.e01; Ft(0,2) = Maa.e02;                     ;  Ft(0,4) = Mav.e01;
	Ft(1,0) = Maa.e10;                  ; Ft(1,2) = Maa.e12;    Ft(1,3) = Mav.e10;
	Ft(2,0) = Maa.e20; Ft(2,1) = Maa.e21;                  ;    Ft(2,3) = Mav.e20;
	                 ;                  ; Ft(0,8) = Map.e02;    Ft(0,9) =-Cnb.e00;  Ft(0,10) =-Cnb.e01; Ft(0,11) =-Cnb.e02; 
	Ft(1,6) = Map.e10;                  ; Ft(1,8) = Map.e12;    Ft(1,9) =-Cnb.e10;  Ft(1,10) =-Cnb.e11; Ft(1,11) =-Cnb.e12; 
	Ft(2,6) = Map.e20;                  ; Ft(2,8) = Map.e22;    Ft(2,9) =-Cnb.e20;  Ft(2,10) =-Cnb.e21; Ft(2,11) =-Cnb.e22; 
	// dv
	                 ; Ft(3,1) = Mva.e01; Ft(3,2) = Mva.e02;    Ft(3,3) = Mvv.e00;  Ft(3,4) = Mvv.e01;  Ft(3,5) = Mvv.e02; 
	Ft(4,0) = Mva.e10;                  ; Ft(4,2) = Mva.e12;    Ft(4,3) = Mvv.e10;  Ft(4,4) = Mvv.e11;  Ft(4,5) = Mvv.e12; 
	Ft(5,0) = Mva.e20; Ft(5,1) = Mva.e21;                  ;    Ft(5,3) = Mvv.e20;  Ft(5,4) = Mvv.e21; 
	Ft(3,6) = Mvp.e00;                  ; Ft(3,8) = Mvp.e02;    Ft(3,12) = Cnb.e00; Ft(3,13) = Cnb.e01; Ft(3,14) = Cnb.e02; 
	Ft(4,6) = Mvp.e10;                  ; Ft(4,8) = Mvp.e12;    Ft(4,12) = Cnb.e10; Ft(4,13) = Cnb.e11; Ft(4,14) = Cnb.e12; 
	Ft(5,6) = Mvp.e20;                  ; Ft(5,8) = Mvp.e22;    Ft(5,12) = Cnb.e20; Ft(5,13) = Cnb.e21; Ft(5,14) = Cnb.e22; 
	// dpos
	                 ; Ft(6,4) = Mpv.e01;                  ;                     ;                    ; Ft(6,8) = Mpp.e02; 
	Ft(7,3) = Mpv.e10;                  ;                  ;    Ft(7,6) = Mpp.e10;                    ; Ft(7,8) = Mpp.e12; 
	                 ;                  ; Ft(8,5) = Mpv.e22;                     ;                    ;                  ;
}

void CKalman::SetHk(void)
{
//	Hk(0,6) = Hk(1,7) = Hk(2,8) = 1.0;
	Hk(0,3) = Hk(1,4) = Hk(2,5) = 1.0; Hk(3,6) = Hk(4,7) = Hk(5,8) = 1.0;
}

/***************************  class CEarth  *********************************/
CEarth::CEarth(double a0, double f0, double g0)
{
	a = a0;	f = f0; wie = glv.wie; 
	b = (1-f)*a;
	e = sqrt(a*a-b*b)/a;	e2 = e*e;
	gn = CVect3(0, 0, -g0);
}

void CEarth::Update(CVect3 &pos, CVect3 &vn)
{
	sl = sin(pos.i), cl = cos(pos.i), tl = sl/cl;
	double sq = 1-e2*sl*sl, sq2 = sqrt(sq);
	RMh = a*(1-e2)/sq/sq2+pos.k;	f_RMh = 1.0/RMh;
	RNh = a/sq2+pos.k;    clRNh = cl*RNh;  f_RNh = 1.0/RNh; f_clRNh = 1.0/clRNh;
	wnie.i = 0,				wnie.j = wie*cl,		wnie.k = wie*sl;
	wnen.i = -vn.j*f_RMh,	wnen.j = vn.i*f_RNh,	wnen.k = wnen.j*tl;
	wnin = wnie + wnen;
	sl2 = sl*sl, sl4 = sl2*sl2;
	gn.k = -( glv.g0*(1+5.27094e-3*sl2+2.32718e-5*sl4)-3.086e-6*pos.k );
	gcc = gn - (wnie+wnin)*vn;
}

/***************************  class CIMU  *********************************/
void CIMU::Update(CVect3 *wm, CVect3 *vm, int nSamples)
{
	static double conefactors[5][4] = {				// coning coefficients
		{2./3},										//2
		{9./20, 27./20},							//3
		{54./105, 92./105, 214./105},				//4
		{250./504, 525./504, 650./504, 1375./504}	//5
		};
	double *pcf = conefactors[nSamples-2];
	CVect3 cm(0.0), sm(0.0), wmm(0.0), vmm(0.0);

	this->nSamples = nSamples;
	if(nSamples==1)  // one-plus-previous sample
	{
		if(prefirst==1) {wm_1=wm[0]; vm_1=vm[0]; prefirst=0;}
		cm = 1.0/12*wm_1; wm_1=wm[0]; 
		sm = 1.0/12*vm_1; vm_1=vm[0];
	}
	if(nSamples>1) prefirst=1;
	for(int i=0; i<nSamples-1; i++)
	{
		cm = cm + pcf[i]*wm[i];
		sm = sm + pcf[i]*vm[i];
		wmm = wmm + wm[i];
		vmm = vmm + vm[i];
	}
	wmm = wmm + wm[i];
	vmm = vmm + vm[i];
	phim = wmm + cm*wm[i];
	dvbm = vmm + 1.0/2*wmm*vmm + (cm*vm[i]+sm*wm[i]);
}

/***************************  class CSINS  *********************************/
CSINS::CSINS(CQuat &qnb0, CVect3 &vn0, CVect3 &pos0)
{
	qnb = qnb0;	vn = vn0, pos = pos0;
	eth.Update(pos0, vn0);
	Cnb = CMat3(qnb); att = CVect3(Cnb); Cnb0 = Cnb;
	Kg = Ka = I33; eb = db = O31;
	wib = fb = fn = an = wnb = web = O31;
}

void CSINS::Update(CVect3 *wm, CVect3 *vm, int nSamples, double ts)
{
	nts = nSamples*ts;
	double nts2 = nts/2;
	imu.Update(wm, vm, nSamples);
	imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // calibration
	CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
	eth.Update(pos01, vn01);
	wib = imu.phim/nts; fb = imu.dvbm/nts;
	web = wib - (~Cnb)*eth.wnie;
	wnb = wib - (qnb*rv2q(imu.phim/2))*eth.wnin;
	fn = qnb*fb;
	an = rv2q(-eth.wnin*nts2)*fn+eth.gcc;
	CVect3 vn1 = vn + an*nts;
	pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
	Cnb0 = Cnb;
	qnb = rv2q(-eth.wnin*nts)*qnb*rv2q(imu.phim);
	Cnb = CMat3(qnb); att = CVect3(Cnb);
}

void CSINS::etm(CMat3 &Maa, CMat3 &Mav, CMat3 &Map, CMat3 &Mva, CMat3 &Mvv, CMat3 &Mvp, CMat3 &Mpv, CMat3 &Mpp)
{
	double tl=eth.tl, secl=1.0/eth.cl, secl2=secl*secl, 
		wN=eth.wnie.j, wU=eth.wnie.k, vE=vn.i, vN=vn.j;
	double f_RMh=eth.f_RMh, f_RNh=eth.f_RNh, f_clRNh=eth.f_clRNh, 
		f_RMh2=f_RMh*f_RMh, f_RNh2=f_RNh*f_RNh;
	CMat3 Avn=askew(vn),
		Mp1(0,0,0, -wU,0,0, wN,0,0),
		Mp2(0,0,vN*f_RMh2, 0,0,-vE*f_RNh2, vE*secl2*f_RNh,0,-vE*tl*f_RNh2);
	Maa = askew(-eth.wnin);
	Mav = CMat3(0,-f_RMh,0, f_RNh,0,0, tl*f_RNh,0,0);
	Map = Mp1+Mp2;
	Mva = askew(fn);
	Mvv = Avn*Mav - askew(eth.wnie+eth.wnin);
	Mvp = Avn*(Mp1+Map);
	double scl = eth.sl*eth.cl;
    Mvp.e20 = Mvp.e20-glv.g0*(5.27094e-3*2*scl+2.32718e-5*4*eth.sl2*scl); Mvp.e22 = Mvp.e22+3.086e-6;
	Mpv = CMat3(0,f_RMh,0, f_clRNh,0,0, 0,0,1);
	Mpp = CMat3(0,0,-vN*f_RMh2, vE*tl*f_clRNh,0,-vE*secl*f_RNh2, 0,0,0);
}

/***************************  class CIMUFile  *********************************/
CIMUFile::CIMUFile(char *fname)
{
	f = fopen(fname, "rt");
	// skip notation
	char prechar=0, curchar=0;
	while(1)
	{
		fscanf(f, "%c", &curchar);
		if(prechar=='\n'&&curchar!='%')	{ fseek(f,-1L,SEEK_CUR); break; }
		prechar = curchar;
	}
	// read data info
	fscanf(f, "%lf %lf %lf %lf %lf %lf", &att0.i, &att0.j, &att0.k, &vn0.i, &vn0.j, &vn0.k);
	fscanf(f, "%lf %lf %lf %lf %lf %lf", &pos0.i, &pos0.j, &pos0.k, &t0, &ts, &g0);
	fscanf(f, "%lf %lf %lf %lf %lf %lf", &gf.i, &gf.j, &gf.k, &af.i, &af.j, &af.k);
	att0 = att0*glv.deg; pos0.i *= glv.deg; pos0.j *= glv.deg; ts /= 1000.0;
	gf = gf*glv.sec; af = af*glv.ug;
	t = t0;
}

int CIMUFile::Skip(int n)
{
	int gx, gy, gz, ax, ay, az;
	for(int i=0; i<n; i++)
	{
		fscanf(f, "%d %d %d %d %d %d", &gx, &gy, &gz, &ax, &ay, &az);
	}
	if(feof(f)) return 0;
	t = t + n*ts;
	return n;
}

int CIMUFile::Load(CVect3 *wm, CVect3 *vm, int n)
{
	int gx, gy, gz, ax, ay, az;
	for(int i=0; i<n; i++)
	{
		fscanf(f, "%d %d %d %d %d %d", &gx, &gy, &gz, &ax, &ay, &az);
		wm[i].i=gx*gf.i; wm[i].j=gy*gf.j; wm[i].k=gz*gf.k; vm[i].i=ax*af.i; vm[i].j=ay*af.j; vm[i].k=az*af.k;
	}
	if(feof(f)) return 0;
	t = t + n*ts;
	if(fmod(t+0.01*ts,50.0)<n*ts)
		printf("%.4lf\n", t);
	return n;
}


/***************************  class AlignCoarse  *********************************/
CMat3 AlignCoarse(CVect3 wmm, CVect3 vmm, double latitude)
{
	double T11, T12, T13, T21, T22, T23, T31, T32, T33;
	double cl = cos(latitude), tl = tan(latitude);
	CVect3 wbib = wmm / norm(wmm),  fb = vmm / norm(vmm);
	T31 = fb.i,				T32 = fb.j,				T33 = fb.k;
	T21 = wbib.i/cl-T31*tl,	T22 = wbib.j/cl-T32*tl,	T23 = wbib.k/cl-T33*tl;   double nn = sqrt(T21*T21+T22*T22+T23*T23);
	T11 = T22*T33-T23*T32,	T12 = T23*T31-T21*T33,	T13 = T21*T32-T22*T31;
	return CMat3(T11, T12, T13, T21, T22, T23, T31, T32, T33);
}

//#define assert(b)
BOOL assert(BOOL b)
{
	int res;

	if(b)
	{
		res = 1;
	}
	else
	{
		res = 0;
	}
	return res;
}

// determine the sign of 'val' with the sensitivity of 'eps'
int signE(double val, double eps)
{
	int s;

	if(val<-eps)
	{
		s = -1;
	}
	else if(val>eps)
	{
		s = 1;
	}
	else
	{
		s = 0; 
	}
	return s;
}

// set double value 'val' between range 'minVal' and 'maxVal'
double range(double val, double minVal, double maxVal)
{
	double res;

	if(val<minVal)
	{ 
		res = minVal; 
	}
	else if(val>maxVal)	
	{ 
		res = maxVal; 
	}
	else				
	{ 
		res = val;
	}
	return res; 
}

double atan2Ex(double y, double x)
{
	double res;

	if((sign(y)==0) && (sign(x)==0))
	{
		res = 0.0;
	}
	else
	{
		res = atan2(y, x);
	}
	return res;
}
