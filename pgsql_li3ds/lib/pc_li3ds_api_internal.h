/***********************************************************************
* pc_li3ds_api_internal.h
*
*  Signatures we need to share within the library, but not for
*  use outside it.
*
*  Copyright (c) 2013 Natural Resources Canada
*  Copyright (c) 2013 OpenGeo
*
***********************************************************************/

#ifndef _PC_LI3DS_API_INTERNAL_H
#define _PC_LI3DS_API_INTERNAL_H

#include "pc_li3ds_api.h"

/**
* Utility defines
*/
#define PC_TRUE 1
#define PC_FALSE 0
#define PC_SUCCESS 1
#define PC_FAILURE 0


/**
* MATRIX
*/

void pc_matrix_43_set(PCMAT43 mat, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff);
void pc_matrix_44_set(PCMAT44 mat, double a,  double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p);
void pc_matrix_33_set_from_quaternion(PCMAT33 mat, double qw, double qx, double qy, double qz);
void pc_matrix_33_multiply_vector_3(PCVEC3 rotatedvec, const PCMAT33 mat, const PCVEC3 vec);
void pc_matrix_43_transform_affine(PCVEC3 res, const PCMAT43 mat, const PCVEC3 vec);
int pc_matrix_44_transform_projective_vector_3(PCVEC3 res, const PCMAT44 mat, const PCVEC3 vec);
void pc_matrix_44_transpose(PCMAT44 res, const PCMAT44 mat);
void pc_matrix_44_multiply_matrix_44(PCMAT44 res, const PCMAT44 mat1, const PCMAT44 mat2);
void pc_matrix_44_multiply_vector_3(PCVEC4 res, const PCMAT44 mat, const PCVEC3 vec);
void pc_matrix_44_multiply_vector_4(PCVEC4 res, const PCMAT44 mat, const PCVEC4 vec);
double pc_matrix_44_determinant(const PCMAT44 m);
void pc_matrix_44_adjugate(PCMAT44 res, const PCMAT44 m, double *determinant);
int pc_matrix_44_inverse(PCMAT44 res, const PCMAT44 m);

void pc_matrix_44_warn(const char *func, int line, const char *name, const PCMAT44 m);
void pc_vector_3_warn(const char *func, int line, const char *name, const PCVEC4 v);
void pc_vector_4_warn(const char *func, int line, const char *name, const PCVEC4 v);

#define PCWARND(x) pcwarn("%s:%d %s=%d",__func__,__LINE__,#x,x);
#define PCWARNLF(x) pcwarn("%s:%d %s=%lf",__func__,__LINE__,#x,x);
#define PCWARNVEC3(x) pc_vector_3_warn(__func__,__LINE__,#x,x);
#define PCWARNVEC4(x) pc_vector_4_warn(__func__,__LINE__,#x,x);
#define PCWARNMAT44(x) pc_matrix_44_warn(__func__,__LINE__,#x,x);

#endif /* _PC_LI3DS_API_INTERNAL_H */
