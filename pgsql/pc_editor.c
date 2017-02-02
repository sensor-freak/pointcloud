/***********************************************************************
* pc_editor.c
*
*  Editor functions for points and patches in PgSQL.
*
*  PgSQL Pointcloud is free and open source software provided
*  by the Government of Canada
*  Copyright (c) 2013 Natural Resources Canada
*  Copyright (c) 2017 Oslandia
*
***********************************************************************/

#include "pc_pgsql.h"      /* Common PgSQL support for our type */

Datum pcpatch_rotate_quaternion(PG_FUNCTION_ARGS);

/**
* Rotate a patch based on a rotation quaternion
* PC_RotateQuaternion(patch pcpatch, qw float8, qx float8, qy float8, qz float8,
*                     xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(pcpatch_rotate_quaternion);
Datum pcpatch_rotate_quaternion(PG_FUNCTION_ARGS)
{
    SERIALIZED_PATCH *serpatch;
    PCPATCH *patch_in, *patch_out;
    PCSCHEMA *schema;
    float8 qw, qx, qy, qz;
    char *xdimname, *ydimname, *zdimname;

    if ( PG_ARGISNULL(0) )
        PG_RETURN_NULL();   /* returns null if no input values */

    serpatch = PG_GETARG_SERPATCH_P(0);
    qw = PG_GETARG_FLOAT8(1);
    qx = PG_GETARG_FLOAT8(2);
    qy = PG_GETARG_FLOAT8(3);
    qz = PG_GETARG_FLOAT8(4);
    xdimname = text_to_cstring(PG_GETARG_TEXT_P(5));
    ydimname = text_to_cstring(PG_GETARG_TEXT_P(6));
    zdimname = text_to_cstring(PG_GETARG_TEXT_P(7));

    schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

    patch_in = pc_patch_deserialize(serpatch, schema);
    if ( ! patch_in )
    {
        elog(ERROR, "failed to deserialize patch");
        PG_RETURN_NULL();
    }

    patch_out = pc_patch_rotate_quaternion(
        patch_in, qw, qx, qy, qz, xdimname, ydimname, zdimname);
    if ( ! patch_out )
    {
        elog(ERROR, "failed to rotate patch");
        PG_RETURN_NULL();
    }

    serpatch = pc_patch_serialize(patch_out, NULL);

    pc_patch_free(patch_in);
    pc_patch_free(patch_out);

    PG_RETURN_POINTER(serpatch);
}

/**
* Rotate a point based on a rotation quaternion
* PC_RotateQuaternion(point pcpoint, qw float8, qx float8, qy float8, qz float8,
*                     xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(pcpoint_rotate_quaternion);
Datum pcpoint_rotate_quaternion(PG_FUNCTION_ARGS)
{
    SERIALIZED_POINT *serpoint;
    PCPOINT *point;
    PCSCHEMA *schema;
    float8 qw, qx, qy, qz;
    char *xdimname, *ydimname, *zdimname;

    if ( PG_ARGISNULL(0) )
        PG_RETURN_NULL();   /* returns null if no input values */

    serpoint = PG_GETARG_SERPOINT_P(0);
    qw = PG_GETARG_FLOAT8(1);
    qx = PG_GETARG_FLOAT8(2);
    qy = PG_GETARG_FLOAT8(3);
    qz = PG_GETARG_FLOAT8(4);
    xdimname = text_to_cstring(PG_GETARG_TEXT_P(5));
    ydimname = text_to_cstring(PG_GETARG_TEXT_P(6));
    zdimname = text_to_cstring(PG_GETARG_TEXT_P(7));

    schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

    point = pc_point_deserialize(serpoint, schema);
    if ( ! point )
    {
        elog(ERROR, "failed to deserialize point");
        PG_RETURN_NULL();
    }

    pc_point_rotate_quaternion(
        point, qw, qx, qy, qz, xdimname, ydimname, zdimname);

    serpoint = pc_point_serialize(point);

    pc_point_free(point);

    PG_RETURN_POINTER(serpoint);
}

/**
* Translate a patch
* PC_Translate(patch pcpatch, tx float8, ty float8, tz float8,
*              xdimname text, ydimname text, zdimname text) returns pcpatch
*/
PG_FUNCTION_INFO_V1(pcpatch_translate);
Datum pcpatch_translate(PG_FUNCTION_ARGS)
{
    SERIALIZED_PATCH *serpatch;
    PCPATCH *patch_in, *patch_out;
    PCSCHEMA *schema;
    float8 tx, ty, tz;
    char *xdimname, *ydimname, *zdimname;

    if ( PG_ARGISNULL(0) )
        PG_RETURN_NULL();   /* returns null if no input values */

    serpatch = PG_GETARG_SERPATCH_P(0);
    tx = PG_GETARG_FLOAT8(1);
    ty = PG_GETARG_FLOAT8(2);
    tz = PG_GETARG_FLOAT8(3);
    xdimname = text_to_cstring(PG_GETARG_TEXT_P(4));
    ydimname = text_to_cstring(PG_GETARG_TEXT_P(5));
    zdimname = text_to_cstring(PG_GETARG_TEXT_P(6));

    schema = pc_schema_from_pcid(serpatch->pcid, fcinfo);

    patch_in = pc_patch_deserialize(serpatch, schema);
    if ( ! patch_in )
    {
        elog(ERROR, "failed to deserialize patch");
        PG_RETURN_NULL();
    }

    patch_out = pc_patch_translate(
        patch_in, tx, ty, tz, xdimname, ydimname, zdimname);
    if ( ! patch_out )
    {
        elog(ERROR, "failed to rotate patch");
        PG_RETURN_NULL();
    }

    serpatch = pc_patch_serialize(patch_out, NULL);

    pc_patch_free(patch_in);
    pc_patch_free(patch_out);

    PG_RETURN_POINTER(serpatch);
}

/**
* Translate a point
* PC_RotateTranslate(point pcpoint, tx float8, ty float8, tz float8,
*                    xdimname text, ydimname text, zdimname text) returns pcpoint
*/
PG_FUNCTION_INFO_V1(pcpoint_translate);
Datum pcpoint_translate(PG_FUNCTION_ARGS)
{
    SERIALIZED_POINT *serpoint;
    PCPOINT *point;
    PCSCHEMA *schema;
    float8 tx, ty, tz;
    char *xdimname, *ydimname, *zdimname;

    if ( PG_ARGISNULL(0) )
        PG_RETURN_NULL();   /* returns null if no input values */

    serpoint = PG_GETARG_SERPOINT_P(0);
    tx = PG_GETARG_FLOAT8(1);
    ty = PG_GETARG_FLOAT8(2);
    tz = PG_GETARG_FLOAT8(3);
    xdimname = text_to_cstring(PG_GETARG_TEXT_P(4));
    ydimname = text_to_cstring(PG_GETARG_TEXT_P(5));
    zdimname = text_to_cstring(PG_GETARG_TEXT_P(6));

    schema = pc_schema_from_pcid(serpoint->pcid, fcinfo);

    point = pc_point_deserialize(serpoint, schema);
    if ( ! point )
    {
        elog(ERROR, "failed to deserialize point");
        PG_RETURN_NULL();
    }

    pc_point_translate(
        point, tx, ty, tz, xdimname, ydimname, zdimname);

    serpoint = pc_point_serialize(point);

    pc_point_free(point);

    PG_RETURN_POINTER(serpoint);
}
