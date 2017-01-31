/***********************************************************************
* pc_matrix.c
*
*  Simple matrix library.
*  Allow this library to be used both inside and outside a
*  PgSQL backend.
*
*  PgSQL Pointcloud is free and open source software provided
*  by the Government of Canada
*  Copyright (c) 2013 Natural Resources Canada
*  Copyright (c) 2017 Oslandia
*
***********************************************************************/

#include "pc_api_internal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
* Sets the 3x3 matrix mat associated to the (qw, qx, qy, qz) quaternion. Assume
* that the quaternion is a unit quaternion.
*/
void
pc_matrix_set_from_quaternion(PCMAT33 mat, double qw, double qx, double qy, double qz)
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
pc_matrix_multiply_vector(PCVEC3 res, const PCMAT33 mat, const PCVEC3 vec)
{
    res[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
    res[1] = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
    res[2] = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
}
