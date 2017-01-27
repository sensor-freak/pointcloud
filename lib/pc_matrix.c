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
* Create the 3x3 matrix associated to the (qw, qx, qy, qz) quaternion. Assume
* that the quaternion is a unit quaternion.
*/
PCMAT33 *
pc_matrix_create_from_quaternion(double qw, double qx, double qy, double qz)
{
    PCMAT33 *mat = pcalloc(sizeof(PCMAT33));
    if ( NULL == mat )
        return NULL;

    (*mat)[0] = 1 - 2 * pow(qy, 2) - 2 * pow(qz, 2);
    (*mat)[1] = 2 * qx * qy - 2 * qw * qz;
    (*mat)[2] = 2 * qx * qz + 2 * qw * qy;

    (*mat)[3] = 2 * qx * qy + 2 * qw * qz;
    (*mat)[4] = 1 - 2 * pow(qx, 2) - 2 * pow(qz, 2);
    (*mat)[5] = 2 * qy * qz - 2 * qw * qx;

    (*mat)[6] = 2 * qx * qz - 2 * qw * qy;
    (*mat)[7] = 2 * qy * qz + 2 * qw * qx;
    (*mat)[8] = 1 - 2 * pow(qx, 2) - 2 * pow(qy, 2);

    return mat;
}

/**
* Multiply mat by vec and store the resuting vector in res.
*/
void
pc_matrix_multiply_vector(const PCMAT33 *mat, const PCVEC3 *vec, PCVEC3 *res)
{
    (*res)[0] = (*mat)[0] * (*vec)[0] +
        (*mat)[1] * (*vec)[1] +
        (*mat)[2] * (*vec)[2];

    (*res)[1] = (*mat)[3] * (*vec)[0] +
        (*mat)[4] * (*vec)[1] +
        (*mat)[5] * (*vec)[2];

    (*res)[2] = (*mat)[6] * (*vec)[0] +
        (*mat)[7] * (*vec)[1] +
        (*mat)[8] * (*vec)[2];
}

void
pc_matrix_cross_product(const PCVEC3 *vec1, const PCVEC3 *vec2, PCVEC3 *res)
{
    (*res)[0] = (*vec1)[1] * (*vec2)[2] - (*vec1[2]) * (*vec2)[1];
    (*res)[1] = (*vec1)[2] * (*vec2)[0] - (*vec1[0]) * (*vec2)[2];
    (*res)[2] = (*vec1)[0] * (*vec2)[1] - (*vec1[1]) * (*vec2)[0];
}

void
pc_matrix_sum(const PCVEC3 *vec1, const PCVEC3 *vec2, PCVEC3 *res)
{
    (*res)[0] = (*vec1)[0] + (*vec2)[0];
    (*res)[1] = (*vec1)[1] + (*vec2)[1];
    (*res)[2] = (*vec1)[2] + (*vec2)[2];
}

void
pc_matrix_scale(const PCVEC3 *vec, double v, PCVEC3 *res)
{
    (*res)[0] = (*vec)[0] * v;
    (*res)[1] = (*vec)[1] * v;
    (*res)[2] = (*vec)[2] * v;
}

void
pc_matrix_rotate_vector(const PCVEC3 *v, double qw, double qx, double qy, double qz, PCVEC3 *res)
{
    // v + 2r x (r x v + w x v)
    PCVEC3 r = {qx, qy, qz};
    PCVEC3 term1, term2, term3, term4, term5;
    pc_matrix_scale(v, qw, &term1);
    pc_matrix_cross_product(&r, v, &term2);
    pc_matrix_sum(&term2, &term1, &term3);
    pc_matrix_scale(&r, 2.0, &term4);
    pc_matrix_cross_product(&term4, &term3, &term5);
    pc_matrix_sum(v, &term5, res);
}
