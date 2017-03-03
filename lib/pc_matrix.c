/***********************************************************************
* pc_matrix.c
*
*  Simple matrix library.
*  Allow this library to be used both inside and outside a
*  PgSQL backend.
*
*  Copyright (c) 2017 Oslandia
*  Copyright (c) 2017 IGN
*
***********************************************************************/

#include "pc_api_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>


void pc_matrix_44_warn(const char *func, int line, const char *name, const PCMAT44 m)
{
	pcwarn("%s:%d %s=\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n%lf %lf %lf %lf\n",
		func, line, name,
		m[ 0], m[ 1], m[ 2], m[ 3],
		m[ 4], m[ 5], m[ 6], m[ 7],
		m[ 8], m[ 9], m[10], m[11],
		m[12], m[13], m[14], m[15]);
}

void pc_vector_3_warn(const char *func, int line, const char *name, const PCVEC4 v)
{
	pcwarn("%s:%d %s=%lf %lf %lf", func, line, name, v[0], v[1], v[2]);
}

void pc_vector_4_warn(const char *func, int line, const char *name, const PCVEC4 v)
{
	pcwarn("%s:%d %s=%lf %lf %lf %lf", func, line, name, v[0], v[1], v[2], v[3]);
}

/**
* Set a 4x3 matrix (4 columns, 3 lines)
*/
void
pc_matrix_43_set(PCMAT43 mat,
		double a, double b, double c, double d,
		double e, double f, double g, double h,
		double i, double j, double k, double l)
{
	mat[0] = a; mat[1] = b; mat[ 2] = c; mat[ 3] = d;
	mat[4] = e; mat[5] = f; mat[ 6] = g; mat[ 7] = h;
	mat[8] = i; mat[9] = j; mat[10] = k; mat[11] = l;
}



/**
* Set a 4x4 matrix (4 columns, 4 lines)
*/
void
pc_matrix_44_copy(PCMAT44 res, const PCMAT44 mat)
{
	memcpy(res, mat, 16*sizeof(double));
}

/**
* Set a 3 vector
*/
void
pc_vector_3_copy(PCVEC3 res, const PCVEC3 vec)
{
	memcpy(res, vec, 3*sizeof(double));
}

void
pc_vector_3_cross(PCVEC3 res, const PCVEC3 v0, const PCVEC3 v1)
{
	res[0] = v0[1]*v1[2] - v0[2]*v1[1];
	res[1] = v0[2]*v1[0] - v0[0]*v1[2];
	res[2] = v0[0]*v1[1] - v0[1]*v1[0];
}

double
pc_vector_3_dot(const PCVEC3 v0, const PCVEC3 v1)
{
	return v0[0]*v1[0] + v0[1]*v1[1] + v0[2]*v1[2];
}

void
pc_vector_3_abs(PCVEC3 res, const PCVEC3 vec)
{
	res[0] = fabs(vec[0]);
	res[1] = fabs(vec[1]);
	res[2] = fabs(vec[2]);
}

/**
* Set a 4x4 matrix.
*/
void
pc_matrix_44_set(PCMAT44 mat,
		double a, double b, double c, double d,
		double e, double f, double g, double h,
		double i, double j, double k, double l,
		double m, double n, double o, double p)
{
	mat[ 0] = a; mat[ 1] = b; mat[ 2] = c; mat[ 3] = d;
	mat[ 4] = e; mat[ 5] = f; mat[ 6] = g; mat[ 7] = h;
	mat[ 8] = i; mat[ 9] = j; mat[10] = k; mat[11] = l;
	mat[12] = m; mat[13] = n; mat[14] = o; mat[15] = p;
}


/**
* Sets the 3x3 matrix mat associated to the (qw, qx, qy, qz) quaternion. Assume
* that the quaternion is a unit quaternion.
*/
void
pc_matrix_33_set_from_quaternion(PCMAT33 mat,
		double qw, double qx, double qy, double qz)
{
	double x = qx+qx;
	double y = qy+qy;
	double z = qz+qz;

	double xx = qx * x;
	double xy = qx * y;
	double xz = qx * z;
	double yy = qy * y;
	double yz = qy * z;
	double zz = qz * z;
	double wx = qw * x;
	double wy = qw * y;
	double wz = qw * z;

	mat[0] = 1 - ( yy + zz );
	mat[1] = xy - wz;
	mat[2] = xz + wy;

	mat[3] = xy + wz;
	mat[4] = 1 - ( xx + zz );
	mat[5] = yz - wx;

	mat[6] = xz - wy;
	mat[7] = yz + wx;
	mat[8] = 1 - ( xx + yy );
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_33_multiply_vector(PCVEC3 res, const PCMAT33 mat, const PCVEC3 vec)
{
	res[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
	res[1] = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
	res[2] = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_44_multiply_vector_3(PCVEC4 res, const PCMAT44 mat, const PCVEC3 vec)
{
	res[0] = mat[ 0] * vec[0] + mat[ 1] * vec[1] + mat[ 2] * vec[2] + mat[ 3];
	res[1] = mat[ 4] * vec[0] + mat[ 5] * vec[1] + mat[ 6] * vec[2] + mat[ 7];
	res[2] = mat[ 8] * vec[0] + mat[ 9] * vec[1] + mat[10] * vec[2] + mat[11];
	res[3] = mat[12] * vec[0] + mat[13] * vec[1] + mat[14] * vec[2] + mat[15];
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_44_multiply_vector_4(PCVEC4 res, const PCMAT44 mat, const PCVEC4 vec)
{
	res[0] = mat[ 0] * vec[0] + mat[ 1] * vec[1] + mat[ 2] * vec[2] + mat[ 3] * vec[3];
	res[1] = mat[ 4] * vec[0] + mat[ 5] * vec[1] + mat[ 6] * vec[2] + mat[ 7] * vec[3];
	res[2] = mat[ 8] * vec[0] + mat[ 9] * vec[1] + mat[10] * vec[2] + mat[11] * vec[3];
	res[3] = mat[12] * vec[0] + mat[13] * vec[1] + mat[14] * vec[2] + mat[15] * vec[3];
}

/**
* Apply an affine transformation to the vector vec and store the resulting
* vector in res.
*/
void
pc_matrix_43_transform_affine(PCVEC3 res, const PCMAT43 mat, const PCVEC3 vec)
{
	res[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[ 2] * vec[2] + mat[ 3];
	res[1] = mat[4] * vec[0] + mat[5] * vec[1] + mat[ 6] * vec[2] + mat[ 7];
	res[2] = mat[8] * vec[0] + mat[9] * vec[1] + mat[10] * vec[2] + mat[11];
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_44_multiply_matrix_44(PCMAT44 res, const PCMAT44 mat1, const PCMAT44 mat2)
{
	PCMAT44 t;
	pc_matrix_44_transpose(t,mat2);
	pc_matrix_44_multiply_vector_4(res   ,t,mat1   );
	pc_matrix_44_multiply_vector_4(res+ 4,t,mat1+ 4);
	pc_matrix_44_multiply_vector_4(res+ 8,t,mat1+ 8);
	pc_matrix_44_multiply_vector_4(res+12,t,mat1+12);
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_44_transpose(PCMAT44 res, const PCMAT44 mat)
{
	res[ 0] = mat[0]; res[ 1] = mat[4]; res[ 2] = mat[ 8]; res[ 3] = mat[12];
	res[ 4] = mat[1]; res[ 5] = mat[5]; res[ 6] = mat[ 9]; res[ 7] = mat[13];
	res[ 8] = mat[2]; res[ 9] = mat[6]; res[10] = mat[10]; res[11] = mat[14];
	res[12] = mat[3]; res[13] = mat[7]; res[14] = mat[11]; res[15] = mat[15];
}

void
pc_matrix_44_multiply(PCMAT44 res, const PCMAT44 mat, double v)
{
	int i;
	for( i = 0; i < 16; ++i)
		res[i] = v*mat[i];
}

void
pc_matrix_44_plucker(double v[6], const PCMAT44 m)
{
	double m00 = m[ 0], m10 = m[ 1];
	double m01 = m[ 4], m11 = m[ 5];
	double m02 = m[ 8], m12 = m[ 9];
	double m03 = m[12], m13 = m[13];

	v[0] = m01*m12-m02*m11;
	v[1] = m00*m12-m02*m10;
	v[2] = m00*m11-m01*m10;
	v[3] = m00*m13-m03*m10;
	v[4] = m01*m13-m03*m11;
	v[5] = m02*m13-m03*m12;
}

double
pc_matrix_44_determinant(const PCMAT44 m)
{
	double p[6];
	double q[6];
	pc_matrix_44_plucker(p,m  ); // 2 left  columns
	pc_matrix_44_plucker(q,m+2); // 2 right columns
	return p[0]*q[3] - p[1]*q[4] + p[2]*q[5] + p[3]*q[0] - p[4]*q[1] + p[5]*q[2];
}

/**
* adjugate is the transposed of the comatrix.
* This is equal to the inverse, up to the scaling by 1/det
* and may be used in place of the inverse in homogeneous contexts.
*/
void
pc_matrix_44_adjugate(PCMAT44 res, const PCMAT44 m, double *determinant)
{
	double p[6];
	double q[6];
	pc_matrix_44_plucker(p,m  ); // 2 left  columns
	pc_matrix_44_plucker(q,m+2); // 2 right columns

	res[ 0] = m[5]*q[5]-m[ 9]*q[4]+m[13]*q[0];
	res[ 4] =-m[4]*q[5]+m[ 8]*q[4]-m[12]*q[0];
	res[ 8] = m[7]*p[5]-m[11]*p[4]+m[15]*p[0];
	res[12] =-m[6]*p[5]+m[10]*p[4]-m[14]*p[0];

	res[ 1] =-m[1]*q[5]+m[ 9]*q[3]-m[13]*q[1];
	res[ 5] = m[0]*q[5]-m[ 8]*q[3]+m[12]*q[1];
	res[ 9] =-m[3]*p[5]+m[11]*p[3]-m[15]*p[1];
	res[13] = m[2]*p[5]-m[10]*p[3]+m[14]*p[1];

	res[ 2] = m[1]*q[4]-m[ 5]*q[3]+m[13]*q[2];
	res[ 6] =-m[0]*q[4]+m[ 4]*q[3]-m[12]*q[2];
	res[10] = m[3]*p[4]-m[ 7]*p[3]+m[15]*p[2];
	res[14] =-m[2]*p[4]+m[ 6]*p[3]-m[14]*p[2];

	res[ 3] =-m[1]*q[0]+m[ 5]*q[1]-m[ 9]*q[2];
	res[ 7] = m[0]*q[0]-m[ 4]*q[1]+m[ 8]*q[2];
	res[11] =-m[3]*p[0]+m[ 7]*p[1]-m[11]*p[2];
	res[15] = m[2]*p[0]-m[ 6]*p[1]+m[10]*p[2];

	if(determinant)
		*determinant = p[0]*q[3] - p[1]*q[4] + p[2]*q[5] + p[3]*q[0] - p[4]*q[1] + p[5]*q[2];
}

/**
* inverse matrix (could be optimized)
*/
int
pc_matrix_44_inverse(PCMAT44 res, const PCMAT44 mat)
{
	double det;
	pc_matrix_44_adjugate(res,mat,&det);
	if(det == 0)
		return PC_FAILURE;
	pc_matrix_44_multiply(res,res,1./det);
	return PC_SUCCESS;
}

void
pc_vector_3_add(PCVEC3 res, const PCVEC3 v1, const PCVEC3 v2)
{
	res[0] = v1[0] + v2[0];
	res[1] = v1[1] + v2[1];
	res[2] = v1[2] + v2[2];
}

void
pc_vector_3_sub(PCVEC3 res, const PCVEC3 v1, const PCVEC3 v2)
{
	res[0] = v1[0] - v2[0];
	res[1] = v1[1] - v2[1];
	res[2] = v1[2] - v2[2];
}

void
pc_vector_3_min(PCVEC3 res, const PCVEC3 v1, const PCVEC3 v2)
{
	res[0] = fmin(v1[0],v2[0]);
	res[1] = fmin(v1[1],v2[1]);
	res[2] = fmin(v1[2],v2[2]);
}

void
pc_vector_3_max(PCVEC3 res, const PCVEC3 v1, const PCVEC3 v2)
{
	res[0] = fmax(v1[0],v2[0]);
	res[1] = fmax(v1[1],v2[1]);
	res[2] = fmax(v1[2],v2[2]);
}

void
pc_vector_4_add(PCVEC4 res, const PCVEC4 v1, const PCVEC4 v2)
{
	res[0] = v1[0] + v2[0];
	res[1] = v1[1] + v2[1];
	res[2] = v1[2] + v2[2];
	res[3] = v1[3] + v2[3];
}

void
pc_vector_4_sub(PCVEC4 res, const PCVEC4 v1, const PCVEC4 v2)
{
	res[0] = v1[0] - v2[0];
	res[1] = v1[1] - v2[1];
	res[2] = v1[2] - v2[2];
	res[3] = v1[3] - v2[3];
}

int
pc_vector_3_from_homogeneous(PCVEC3 res, const PCVEC4 v)
{
	if(v[3]==0) return PC_FAILURE;
	res[0] = v[0] / v[3];
	res[1] = v[1] / v[3];
	res[2] = v[2] / v[3];
	return PC_SUCCESS;
}

#define PCVEC2MID(x,y) 0.5*((x)[0]+(y)[0]), 0.5*((x)[1]+(y)[1])
#define PCVEC3SUB(x,y) (x)[0]-(y)[0], (x)[1]-(y)[1], (x)[2]-(y)[2]
#define PCVEC3ADD(x,y) (x)[0]+(y)[0], (x)[1]+(y)[1], (x)[2]+(y)[2]
#define PCVEC4SUB(x,y) (x)[0]-(y)[0], (x)[1]-(y)[1], (x)[2]-(y)[2], (x)[3]-(y)[3]
#define PCVEC4ADD(x,y) (x)[0]+(y)[0], (x)[1]+(y)[1], (x)[2]+(y)[2], (x)[3]+(y)[3]

/**
* Apply a projective transformation to the vector vec and store the resulting
* vector in res.
*/
int
pc_matrix_44_transform_projective_vector_3(PCVEC3 res, const PCMAT44 mat, const PCVEC3 vec)
{
	PCVEC4 hres;
	pc_matrix_44_multiply_vector_3(hres,mat,vec);
	return pc_vector_3_from_homogeneous(res,hres);
}

int
pc_matrix_44_transform_projective_vector_4(PCVEC3 res, const PCMAT44 mat, const PCVEC4 vec)
{
	PCVEC4 hres;
	pc_matrix_44_multiply_vector_4(hres,mat,vec);
	return pc_vector_3_from_homogeneous(res,hres);
}

/**
* vector3 normalization
*/

int
pc_vector_3_normalize(PCVEC3 res, const PCVEC3 vec)
{
	double r2 = pc_vector_3_dot(vec,vec);
	if(r2<=0)
		return PC_FAILURE;
	double inv = 1/sqrt(r2);
	res[0] = vec[0] * inv;
	res[1] = vec[1] * inv;
	res[2] = vec[2] * inv;
	return PC_SUCCESS;
}

/**
* Cartesian <- Spherical
*/

int
pc_vector_3_cartesian_from_spherical(PCVEC3 res, const PCVEC3 vec)
{
	double rc2 = vec[0]*cos(vec[2]);
	double rs2 = vec[0]*sin(vec[2]);
	res[0] = rc2*cos(vec[1]);
	res[1] = rc2*sin(vec[1]);
	res[2] = rs2;
	return PC_SUCCESS;
}

/**
* Spherical <- Cartesian
*/

int
pc_vector_3_spherical_from_cartesian(PCVEC3 res, const PCVEC3 vec)
{
	res[0] = sqrt(pc_vector_3_dot(vec,vec));
	res[1] = atan2(vec[1],vec[0]);
	res[2] = asin (vec[2]/res[0]);
	return PC_SUCCESS;
}

/**
* Distorsion polynom : r3,r5,r7
* https://github.com/IGNF/libOri/blob/master/src/DistortionPolynom.cpp
*/

int
pc_vector_3_distorsion(PCVEC3 res, const PCDISTORSION *d, const PCVEC3 vec)
{
	double v[] = { res[0]-d->pps[0], res[0]-d->pps[1] };
	double r2 = v[0]*v[0]+v[1]*v[1];
	double dr;
	if(r2>d->r2max)
		return PC_FAILURE;

	dr = r2*(d->c[0] + r2*(d->c[1] + r2*d->c[2]));
	res[0] = vec[0] + dr*v[0];
	res[1] = vec[1] + dr*v[1];
	res[2] = vec[2];
	return PC_SUCCESS;
}

/**
* Undistorsion : polynom r3,r5,r7
* https://github.com/IGNF/libOri/blob/master/src/DistortionPolynom.cpp
*/

#define PCUNDISTORSION_ERR2MAX (1e-15)

int
pc_vector_3_undistorsion(PCVEC3 res, const PCDISTORSION *d, const PCVEC3 vec)
{

	double v[] = { res[0]-d->pps[0], res[0]-d->pps[1] };
	double r2 = v[0]*v[0]+v[1]*v[1];
	double r4 = r2*r2;
	double r6 = r4*r2;

	double c3 = d->c[0]*r2, d2 = 3*c3;
	double c5 = d->c[1]*r4, d4 = 5*c5;
	double c7 = d->c[2]*r6, d6 = 7*c7;

	double t = 1; // Newton's method initialisation
	int i, i_max=50; // max num of iterations
	for( i = 0; i < i_max; ++i )
	{
		double t2 = t *t;
		double R_, R  = -1+t+t*t2*(c3+t2*(c5+t2*c7));

		if(R*R*r2<PCUNDISTORSION_ERR2MAX && t2*r2<d->r2max)
		{
			res[0] = d->pps[0] + t * v[0];
			res[1] = d->pps[1] + t * v[1];
			res[2] = vec[2];
			return PC_SUCCESS;
		}
		R_ = 1+  t2*(d2+t2*(d4+t2*d6));
		t -= R/R_; // Newton's method iteration
		if(t<0) t=0;
	}
	return PC_FAILURE;
}
