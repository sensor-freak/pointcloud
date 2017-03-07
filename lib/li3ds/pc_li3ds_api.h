/***********************************************************************
* pc_li3ds_api.h
*
*  Structures and function signatures for point clouds
*
***********************************************************************/

#ifndef _PC_LI3DS_API_H
#define _PC_LI3DS_API_H

#include "../pc_api.h"

/* Linear Algebra types */
typedef double PCVEC3[3];
typedef double PCVEC4[4];
typedef double PCMAT33[9];
typedef double PCMAT43[12];
typedef double PCMAT44[16];
typedef PCVEC3 PCBOX3[2];

typedef struct
{
	PCMAT44 fwd;
	double *bwd;
	double det;
} PCFRUSTUM;

typedef struct
{
	double pps[2];
	double c[3];
	double r2max;
} PCDISTORSION;


/** rotate a patch given a unit quaternion */
PCPATCH *pc_patch_rotate_quaternion(const PCPATCH *patch, double qw, double qx, double qy, double qz, const char *xdimname, const char *ydimname, const char *zdimname);

/** rotate a point in place given a unit quaternion */
void pc_point_rotate_quaternion(PCPOINT *point, double qw, double qx, double qy, double qz, const char *xdimname, const char *ydimname, const char *zdimname);

/** translate a patch */
PCPATCH *pc_patch_translate(const PCPATCH *patch, double tx, double ty, double tz, const char *xdimname, const char *ydimname, const char *zdimname);

/** translate a point */
void pc_point_translate(PCPOINT *point, double tx, double ty, double tz, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply an affine transformation to a patch */
PCPATCH *pc_patch_affine(const PCPATCH *patch, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply an affine transformation to a point */
void pc_point_affine(PCPOINT *point, double a, double b, double c, double d, double e, double f, double g, double h, double i, double xoff, double yoff, double zoff, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply a projective transformation to a patch */
PCPATCH *pc_patch_projective(const PCPATCH *patch, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p, const char *xdimname, const char *ydimname, const char *zdimname);

/** apply a projective transformation to a point */
void pc_point_projective(PCPOINT *point, double a, double b, double c, double d, double e, double f, double g, double h, double i, double j, double k, double l, double m, double n, double o, double p, const char *xdimname, const char *ydimname, const char *zdimname);

/** convert a box to a frustum (lossless) */
int pc_frustum_from_box(PCFRUSTUM *res, const PCBOX3 b);

/** convert a frustum to a box (lossy if not axis-aligned) */
int  pc_box_from_frustum(PCBOX3 res, const PCFRUSTUM *f);

/** convert a frustum to a postgis ewkb byte array */
uint8_t *pc_frustum_to_geometry_wkb(const PCFRUSTUM *f, uint32_t srid, size_t *wkbsize);

/** test frustum intersection. valid modes are 1,2,3,4 */
int pc_frustum_intersects(const PCFRUSTUM *f1, const PCFRUSTUM *f2, uint8_t mode);

/** test frustum containment */
int pc_frustum_contains(const PCFRUSTUM *f1, const PCFRUSTUM *f2);

/** test frustum validity */
int pc_frustum_is_valid(const PCFRUSTUM *f);

/** try to make the frustum valid */
int pc_frustum_make_valid(PCFRUSTUM *res, const PCFRUSTUM *f);

/** 3D volume **/
double pc_box_volume(const PCBOX3 b);

/** 3D volume, f should be valid **/
double pc_frustum_volume(const PCFRUSTUM *f);

#endif
