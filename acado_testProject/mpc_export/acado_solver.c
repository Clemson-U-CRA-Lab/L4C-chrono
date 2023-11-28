/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 5 + 4];

acadoWorkspace.state[40] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.state[41] = acadoVariables.u[lRun1 * 2 + 1];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 5] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 5 + 5];
acadoWorkspace.d[lRun1 * 5 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 5 + 6];
acadoWorkspace.d[lRun1 * 5 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 5 + 7];
acadoWorkspace.d[lRun1 * 5 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 5 + 8];
acadoWorkspace.d[lRun1 * 5 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 5 + 9];

acadoWorkspace.evGx[lRun1 * 25] = acadoWorkspace.state[5];
acadoWorkspace.evGx[lRun1 * 25 + 1] = acadoWorkspace.state[6];
acadoWorkspace.evGx[lRun1 * 25 + 2] = acadoWorkspace.state[7];
acadoWorkspace.evGx[lRun1 * 25 + 3] = acadoWorkspace.state[8];
acadoWorkspace.evGx[lRun1 * 25 + 4] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 25 + 5] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 25 + 6] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 25 + 7] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 25 + 8] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 25 + 9] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 25 + 10] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 25 + 11] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 25 + 12] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 25 + 13] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 25 + 14] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 25 + 15] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 25 + 16] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 25 + 17] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 25 + 18] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 25 + 19] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 25 + 20] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 25 + 21] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 25 + 22] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 25 + 23] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 25 + 24] = acadoWorkspace.state[29];

acadoWorkspace.evGu[lRun1 * 10] = acadoWorkspace.state[30];
acadoWorkspace.evGu[lRun1 * 10 + 1] = acadoWorkspace.state[31];
acadoWorkspace.evGu[lRun1 * 10 + 2] = acadoWorkspace.state[32];
acadoWorkspace.evGu[lRun1 * 10 + 3] = acadoWorkspace.state[33];
acadoWorkspace.evGu[lRun1 * 10 + 4] = acadoWorkspace.state[34];
acadoWorkspace.evGu[lRun1 * 10 + 5] = acadoWorkspace.state[35];
acadoWorkspace.evGu[lRun1 * 10 + 6] = acadoWorkspace.state[36];
acadoWorkspace.evGu[lRun1 * 10 + 7] = acadoWorkspace.state[37];
acadoWorkspace.evGu[lRun1 * 10 + 8] = acadoWorkspace.state[38];
acadoWorkspace.evGu[lRun1 * 10 + 9] = acadoWorkspace.state[39];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
const real_t* od = in + 7;
/* Vector of auxiliary variables; number of elements: 16. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[1])*(xd[1]-od[1])));
a[1] = (((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[3])*(xd[1]-od[3])));
a[2] = ((xd[0]-od[0])+(xd[0]-od[0]));
a[3] = ((xd[1]-od[1])+(xd[1]-od[1]));
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = ((xd[0]-od[0])+(xd[0]-od[0]));
a[8] = ((xd[1]-od[3])+(xd[1]-od[3]));
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (real_t)(0.0000000000000000e+00);
a[15] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (xd[0]-(real_t)(1.0000000000000000e+00));
out[1] = (xd[1]-(real_t)(1.0000000000000000e+00));
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = a[0];
out[6] = a[1];
out[7] = u[0];
out[8] = u[1];
out[9] = u[2];
out[10] = u[3];
out[11] = (real_t)(1.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(1.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(1.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(1.0000000000000000e+00);
out[36] = a[2];
out[37] = a[3];
out[38] = a[4];
out[39] = a[5];
out[40] = a[6];
out[41] = a[7];
out[42] = a[8];
out[43] = a[9];
out[44] = a[10];
out[45] = a[11];
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = a[12];
out[77] = a[13];
out[78] = a[14];
out[79] = a[15];
out[80] = (real_t)(1.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(1.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 5;
/* Vector of auxiliary variables; number of elements: 12. */
real_t* a = acadoWorkspace.objAuxVar;

/* Compute intermediate quantities: */
a[0] = (((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[1])*(xd[1]-od[1])));
a[1] = (((xd[0]-od[0])*(xd[0]-od[0]))+((xd[1]-od[3])*(xd[1]-od[3])));
a[2] = ((xd[0]-od[0])+(xd[0]-od[0]));
a[3] = ((xd[1]-od[1])+(xd[1]-od[1]));
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = (real_t)(0.0000000000000000e+00);
a[6] = (real_t)(0.0000000000000000e+00);
a[7] = ((xd[0]-od[0])+(xd[0]-od[0]));
a[8] = ((xd[1]-od[3])+(xd[1]-od[3]));
a[9] = (real_t)(0.0000000000000000e+00);
a[10] = (real_t)(0.0000000000000000e+00);
a[11] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = a[0];
out[6] = a[1];
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(1.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(1.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(1.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(1.0000000000000000e+00);
out[32] = a[2];
out[33] = a[3];
out[34] = a[4];
out[35] = a[5];
out[36] = a[6];
out[37] = a[7];
out[38] = a[8];
out[39] = a[9];
out[40] = a[10];
out[41] = a[11];
}

void acado_evaluatePathConstraints(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 5;
/* Vector of auxiliary variables; number of elements: 48. */
real_t* a = acadoWorkspace.conAuxVar;

/* Compute intermediate quantities: */
a[0] = (tan(xd[3]));
a[1] = (((xd[4]*xd[4])/(real_t)(2.7500000000000000e+00))*a[0]);
a[2] = (real_t)(0.0000000000000000e+00);
a[3] = (real_t)(0.0000000000000000e+00);
a[4] = (real_t)(0.0000000000000000e+00);
a[5] = ((real_t)(1.0000000000000000e+00)/(pow((cos(xd[3])),2)));
a[6] = ((xd[4]*xd[4])/(real_t)(2.7500000000000000e+00));
a[7] = (a[5]*a[6]);
a[8] = ((real_t)(1.0000000000000000e+00)/(real_t)(2.7500000000000000e+00));
a[9] = ((xd[4]+xd[4])*a[8]);
a[10] = (a[9]*a[0]);
a[11] = (real_t)(0.0000000000000000e+00);
a[12] = (real_t)(0.0000000000000000e+00);
a[13] = (real_t)(0.0000000000000000e+00);
a[14] = (a[5]*a[6]);
a[15] = (a[9]*a[0]);
a[16] = (real_t)(0.0000000000000000e+00);
a[17] = (real_t)(0.0000000000000000e+00);
a[18] = (real_t)(0.0000000000000000e+00);
a[19] = (real_t)(0.0000000000000000e+00);
a[20] = (real_t)(0.0000000000000000e+00);
a[21] = (real_t)(0.0000000000000000e+00);
a[22] = (real_t)(0.0000000000000000e+00);
a[23] = (real_t)(0.0000000000000000e+00);
a[24] = (real_t)(0.0000000000000000e+00);
a[25] = (real_t)(0.0000000000000000e+00);
a[26] = (real_t)(0.0000000000000000e+00);
a[27] = (real_t)(0.0000000000000000e+00);
a[28] = (real_t)(0.0000000000000000e+00);
a[29] = (real_t)(1.0000000000000000e+00);
a[30] = (real_t)(0.0000000000000000e+00);
a[31] = (real_t)(0.0000000000000000e+00);
a[32] = (real_t)(0.0000000000000000e+00);
a[33] = (real_t)(0.0000000000000000e+00);
a[34] = (real_t)(1.0000000000000000e+00);
a[35] = (real_t)(0.0000000000000000e+00);
a[36] = (real_t)(0.0000000000000000e+00);
a[37] = (real_t)(0.0000000000000000e+00);
a[38] = (real_t)(0.0000000000000000e+00);
a[39] = (real_t)(0.0000000000000000e+00);
a[40] = (real_t)(0.0000000000000000e+00);
a[41] = (real_t)(1.0000000000000000e+00);
a[42] = (real_t)(0.0000000000000000e+00);
a[43] = (real_t)(1.0000000000000000e+00);
a[44] = (real_t)(0.0000000000000000e+00);
a[45] = (real_t)(0.0000000000000000e+00);
a[46] = (real_t)(0.0000000000000000e+00);
a[47] = (real_t)(0.0000000000000000e+00);

/* Compute outputs: */
out[0] = (a[1]+u[2]);
out[1] = (a[1]-u[2]);
out[2] = (u[1]+u[2]);
out[3] = (u[1]-u[2]);
out[4] = (xd[3]+u[3]);
out[5] = (xd[3]-u[3]);
out[6] = a[2];
out[7] = a[3];
out[8] = a[4];
out[9] = a[7];
out[10] = a[10];
out[11] = a[11];
out[12] = a[12];
out[13] = a[13];
out[14] = a[14];
out[15] = a[15];
out[16] = a[16];
out[17] = a[17];
out[18] = a[18];
out[19] = a[19];
out[20] = a[20];
out[21] = a[21];
out[22] = a[22];
out[23] = a[23];
out[24] = a[24];
out[25] = a[25];
out[26] = a[26];
out[27] = a[27];
out[28] = a[28];
out[29] = a[29];
out[30] = a[30];
out[31] = a[31];
out[32] = a[32];
out[33] = a[33];
out[34] = a[34];
out[35] = a[35];
out[36] = a[36];
out[37] = a[37];
out[38] = a[38];
out[39] = a[39];
out[40] = a[40];
out[41] = a[41];
out[42] = a[42];
out[43] = a[43];
out[44] = a[44];
out[45] = a[45];
out[46] = a[46];
out[47] = a[47];
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = + tmpFx[0];
tmpQ2[1] = + tmpFx[5];
tmpQ2[2] = + tmpFx[10];
tmpQ2[3] = + tmpFx[15];
tmpQ2[4] = + tmpFx[20];
tmpQ2[5] = + tmpFx[25];
tmpQ2[6] = + tmpFx[30];
tmpQ2[7] = + tmpFx[35];
tmpQ2[8] = + tmpFx[40];
tmpQ2[9] = + tmpFx[45];
tmpQ2[10] = + tmpFx[50];
tmpQ2[11] = + tmpFx[1];
tmpQ2[12] = + tmpFx[6];
tmpQ2[13] = + tmpFx[11];
tmpQ2[14] = + tmpFx[16];
tmpQ2[15] = + tmpFx[21];
tmpQ2[16] = + tmpFx[26];
tmpQ2[17] = + tmpFx[31];
tmpQ2[18] = + tmpFx[36];
tmpQ2[19] = + tmpFx[41];
tmpQ2[20] = + tmpFx[46];
tmpQ2[21] = + tmpFx[51];
tmpQ2[22] = + tmpFx[2];
tmpQ2[23] = + tmpFx[7];
tmpQ2[24] = + tmpFx[12];
tmpQ2[25] = + tmpFx[17];
tmpQ2[26] = + tmpFx[22];
tmpQ2[27] = + tmpFx[27];
tmpQ2[28] = + tmpFx[32];
tmpQ2[29] = + tmpFx[37];
tmpQ2[30] = + tmpFx[42];
tmpQ2[31] = + tmpFx[47];
tmpQ2[32] = + tmpFx[52];
tmpQ2[33] = + tmpFx[3];
tmpQ2[34] = + tmpFx[8];
tmpQ2[35] = + tmpFx[13];
tmpQ2[36] = + tmpFx[18];
tmpQ2[37] = + tmpFx[23];
tmpQ2[38] = + tmpFx[28];
tmpQ2[39] = + tmpFx[33];
tmpQ2[40] = + tmpFx[38];
tmpQ2[41] = + tmpFx[43];
tmpQ2[42] = + tmpFx[48];
tmpQ2[43] = + tmpFx[53];
tmpQ2[44] = + tmpFx[4];
tmpQ2[45] = + tmpFx[9];
tmpQ2[46] = + tmpFx[14];
tmpQ2[47] = + tmpFx[19];
tmpQ2[48] = + tmpFx[24];
tmpQ2[49] = + tmpFx[29];
tmpQ2[50] = + tmpFx[34];
tmpQ2[51] = + tmpFx[39];
tmpQ2[52] = + tmpFx[44];
tmpQ2[53] = + tmpFx[49];
tmpQ2[54] = + tmpFx[54];
tmpQ1[0] = + tmpQ2[0]*tmpFx[0] + tmpQ2[1]*tmpFx[5] + tmpQ2[2]*tmpFx[10] + tmpQ2[3]*tmpFx[15] + tmpQ2[4]*tmpFx[20] + tmpQ2[5]*tmpFx[25] + tmpQ2[6]*tmpFx[30] + tmpQ2[7]*tmpFx[35] + tmpQ2[8]*tmpFx[40] + tmpQ2[9]*tmpFx[45] + tmpQ2[10]*tmpFx[50];
tmpQ1[1] = + tmpQ2[0]*tmpFx[1] + tmpQ2[1]*tmpFx[6] + tmpQ2[2]*tmpFx[11] + tmpQ2[3]*tmpFx[16] + tmpQ2[4]*tmpFx[21] + tmpQ2[5]*tmpFx[26] + tmpQ2[6]*tmpFx[31] + tmpQ2[7]*tmpFx[36] + tmpQ2[8]*tmpFx[41] + tmpQ2[9]*tmpFx[46] + tmpQ2[10]*tmpFx[51];
tmpQ1[2] = + tmpQ2[0]*tmpFx[2] + tmpQ2[1]*tmpFx[7] + tmpQ2[2]*tmpFx[12] + tmpQ2[3]*tmpFx[17] + tmpQ2[4]*tmpFx[22] + tmpQ2[5]*tmpFx[27] + tmpQ2[6]*tmpFx[32] + tmpQ2[7]*tmpFx[37] + tmpQ2[8]*tmpFx[42] + tmpQ2[9]*tmpFx[47] + tmpQ2[10]*tmpFx[52];
tmpQ1[3] = + tmpQ2[0]*tmpFx[3] + tmpQ2[1]*tmpFx[8] + tmpQ2[2]*tmpFx[13] + tmpQ2[3]*tmpFx[18] + tmpQ2[4]*tmpFx[23] + tmpQ2[5]*tmpFx[28] + tmpQ2[6]*tmpFx[33] + tmpQ2[7]*tmpFx[38] + tmpQ2[8]*tmpFx[43] + tmpQ2[9]*tmpFx[48] + tmpQ2[10]*tmpFx[53];
tmpQ1[4] = + tmpQ2[0]*tmpFx[4] + tmpQ2[1]*tmpFx[9] + tmpQ2[2]*tmpFx[14] + tmpQ2[3]*tmpFx[19] + tmpQ2[4]*tmpFx[24] + tmpQ2[5]*tmpFx[29] + tmpQ2[6]*tmpFx[34] + tmpQ2[7]*tmpFx[39] + tmpQ2[8]*tmpFx[44] + tmpQ2[9]*tmpFx[49] + tmpQ2[10]*tmpFx[54];
tmpQ1[5] = + tmpQ2[11]*tmpFx[0] + tmpQ2[12]*tmpFx[5] + tmpQ2[13]*tmpFx[10] + tmpQ2[14]*tmpFx[15] + tmpQ2[15]*tmpFx[20] + tmpQ2[16]*tmpFx[25] + tmpQ2[17]*tmpFx[30] + tmpQ2[18]*tmpFx[35] + tmpQ2[19]*tmpFx[40] + tmpQ2[20]*tmpFx[45] + tmpQ2[21]*tmpFx[50];
tmpQ1[6] = + tmpQ2[11]*tmpFx[1] + tmpQ2[12]*tmpFx[6] + tmpQ2[13]*tmpFx[11] + tmpQ2[14]*tmpFx[16] + tmpQ2[15]*tmpFx[21] + tmpQ2[16]*tmpFx[26] + tmpQ2[17]*tmpFx[31] + tmpQ2[18]*tmpFx[36] + tmpQ2[19]*tmpFx[41] + tmpQ2[20]*tmpFx[46] + tmpQ2[21]*tmpFx[51];
tmpQ1[7] = + tmpQ2[11]*tmpFx[2] + tmpQ2[12]*tmpFx[7] + tmpQ2[13]*tmpFx[12] + tmpQ2[14]*tmpFx[17] + tmpQ2[15]*tmpFx[22] + tmpQ2[16]*tmpFx[27] + tmpQ2[17]*tmpFx[32] + tmpQ2[18]*tmpFx[37] + tmpQ2[19]*tmpFx[42] + tmpQ2[20]*tmpFx[47] + tmpQ2[21]*tmpFx[52];
tmpQ1[8] = + tmpQ2[11]*tmpFx[3] + tmpQ2[12]*tmpFx[8] + tmpQ2[13]*tmpFx[13] + tmpQ2[14]*tmpFx[18] + tmpQ2[15]*tmpFx[23] + tmpQ2[16]*tmpFx[28] + tmpQ2[17]*tmpFx[33] + tmpQ2[18]*tmpFx[38] + tmpQ2[19]*tmpFx[43] + tmpQ2[20]*tmpFx[48] + tmpQ2[21]*tmpFx[53];
tmpQ1[9] = + tmpQ2[11]*tmpFx[4] + tmpQ2[12]*tmpFx[9] + tmpQ2[13]*tmpFx[14] + tmpQ2[14]*tmpFx[19] + tmpQ2[15]*tmpFx[24] + tmpQ2[16]*tmpFx[29] + tmpQ2[17]*tmpFx[34] + tmpQ2[18]*tmpFx[39] + tmpQ2[19]*tmpFx[44] + tmpQ2[20]*tmpFx[49] + tmpQ2[21]*tmpFx[54];
tmpQ1[10] = + tmpQ2[22]*tmpFx[0] + tmpQ2[23]*tmpFx[5] + tmpQ2[24]*tmpFx[10] + tmpQ2[25]*tmpFx[15] + tmpQ2[26]*tmpFx[20] + tmpQ2[27]*tmpFx[25] + tmpQ2[28]*tmpFx[30] + tmpQ2[29]*tmpFx[35] + tmpQ2[30]*tmpFx[40] + tmpQ2[31]*tmpFx[45] + tmpQ2[32]*tmpFx[50];
tmpQ1[11] = + tmpQ2[22]*tmpFx[1] + tmpQ2[23]*tmpFx[6] + tmpQ2[24]*tmpFx[11] + tmpQ2[25]*tmpFx[16] + tmpQ2[26]*tmpFx[21] + tmpQ2[27]*tmpFx[26] + tmpQ2[28]*tmpFx[31] + tmpQ2[29]*tmpFx[36] + tmpQ2[30]*tmpFx[41] + tmpQ2[31]*tmpFx[46] + tmpQ2[32]*tmpFx[51];
tmpQ1[12] = + tmpQ2[22]*tmpFx[2] + tmpQ2[23]*tmpFx[7] + tmpQ2[24]*tmpFx[12] + tmpQ2[25]*tmpFx[17] + tmpQ2[26]*tmpFx[22] + tmpQ2[27]*tmpFx[27] + tmpQ2[28]*tmpFx[32] + tmpQ2[29]*tmpFx[37] + tmpQ2[30]*tmpFx[42] + tmpQ2[31]*tmpFx[47] + tmpQ2[32]*tmpFx[52];
tmpQ1[13] = + tmpQ2[22]*tmpFx[3] + tmpQ2[23]*tmpFx[8] + tmpQ2[24]*tmpFx[13] + tmpQ2[25]*tmpFx[18] + tmpQ2[26]*tmpFx[23] + tmpQ2[27]*tmpFx[28] + tmpQ2[28]*tmpFx[33] + tmpQ2[29]*tmpFx[38] + tmpQ2[30]*tmpFx[43] + tmpQ2[31]*tmpFx[48] + tmpQ2[32]*tmpFx[53];
tmpQ1[14] = + tmpQ2[22]*tmpFx[4] + tmpQ2[23]*tmpFx[9] + tmpQ2[24]*tmpFx[14] + tmpQ2[25]*tmpFx[19] + tmpQ2[26]*tmpFx[24] + tmpQ2[27]*tmpFx[29] + tmpQ2[28]*tmpFx[34] + tmpQ2[29]*tmpFx[39] + tmpQ2[30]*tmpFx[44] + tmpQ2[31]*tmpFx[49] + tmpQ2[32]*tmpFx[54];
tmpQ1[15] = + tmpQ2[33]*tmpFx[0] + tmpQ2[34]*tmpFx[5] + tmpQ2[35]*tmpFx[10] + tmpQ2[36]*tmpFx[15] + tmpQ2[37]*tmpFx[20] + tmpQ2[38]*tmpFx[25] + tmpQ2[39]*tmpFx[30] + tmpQ2[40]*tmpFx[35] + tmpQ2[41]*tmpFx[40] + tmpQ2[42]*tmpFx[45] + tmpQ2[43]*tmpFx[50];
tmpQ1[16] = + tmpQ2[33]*tmpFx[1] + tmpQ2[34]*tmpFx[6] + tmpQ2[35]*tmpFx[11] + tmpQ2[36]*tmpFx[16] + tmpQ2[37]*tmpFx[21] + tmpQ2[38]*tmpFx[26] + tmpQ2[39]*tmpFx[31] + tmpQ2[40]*tmpFx[36] + tmpQ2[41]*tmpFx[41] + tmpQ2[42]*tmpFx[46] + tmpQ2[43]*tmpFx[51];
tmpQ1[17] = + tmpQ2[33]*tmpFx[2] + tmpQ2[34]*tmpFx[7] + tmpQ2[35]*tmpFx[12] + tmpQ2[36]*tmpFx[17] + tmpQ2[37]*tmpFx[22] + tmpQ2[38]*tmpFx[27] + tmpQ2[39]*tmpFx[32] + tmpQ2[40]*tmpFx[37] + tmpQ2[41]*tmpFx[42] + tmpQ2[42]*tmpFx[47] + tmpQ2[43]*tmpFx[52];
tmpQ1[18] = + tmpQ2[33]*tmpFx[3] + tmpQ2[34]*tmpFx[8] + tmpQ2[35]*tmpFx[13] + tmpQ2[36]*tmpFx[18] + tmpQ2[37]*tmpFx[23] + tmpQ2[38]*tmpFx[28] + tmpQ2[39]*tmpFx[33] + tmpQ2[40]*tmpFx[38] + tmpQ2[41]*tmpFx[43] + tmpQ2[42]*tmpFx[48] + tmpQ2[43]*tmpFx[53];
tmpQ1[19] = + tmpQ2[33]*tmpFx[4] + tmpQ2[34]*tmpFx[9] + tmpQ2[35]*tmpFx[14] + tmpQ2[36]*tmpFx[19] + tmpQ2[37]*tmpFx[24] + tmpQ2[38]*tmpFx[29] + tmpQ2[39]*tmpFx[34] + tmpQ2[40]*tmpFx[39] + tmpQ2[41]*tmpFx[44] + tmpQ2[42]*tmpFx[49] + tmpQ2[43]*tmpFx[54];
tmpQ1[20] = + tmpQ2[44]*tmpFx[0] + tmpQ2[45]*tmpFx[5] + tmpQ2[46]*tmpFx[10] + tmpQ2[47]*tmpFx[15] + tmpQ2[48]*tmpFx[20] + tmpQ2[49]*tmpFx[25] + tmpQ2[50]*tmpFx[30] + tmpQ2[51]*tmpFx[35] + tmpQ2[52]*tmpFx[40] + tmpQ2[53]*tmpFx[45] + tmpQ2[54]*tmpFx[50];
tmpQ1[21] = + tmpQ2[44]*tmpFx[1] + tmpQ2[45]*tmpFx[6] + tmpQ2[46]*tmpFx[11] + tmpQ2[47]*tmpFx[16] + tmpQ2[48]*tmpFx[21] + tmpQ2[49]*tmpFx[26] + tmpQ2[50]*tmpFx[31] + tmpQ2[51]*tmpFx[36] + tmpQ2[52]*tmpFx[41] + tmpQ2[53]*tmpFx[46] + tmpQ2[54]*tmpFx[51];
tmpQ1[22] = + tmpQ2[44]*tmpFx[2] + tmpQ2[45]*tmpFx[7] + tmpQ2[46]*tmpFx[12] + tmpQ2[47]*tmpFx[17] + tmpQ2[48]*tmpFx[22] + tmpQ2[49]*tmpFx[27] + tmpQ2[50]*tmpFx[32] + tmpQ2[51]*tmpFx[37] + tmpQ2[52]*tmpFx[42] + tmpQ2[53]*tmpFx[47] + tmpQ2[54]*tmpFx[52];
tmpQ1[23] = + tmpQ2[44]*tmpFx[3] + tmpQ2[45]*tmpFx[8] + tmpQ2[46]*tmpFx[13] + tmpQ2[47]*tmpFx[18] + tmpQ2[48]*tmpFx[23] + tmpQ2[49]*tmpFx[28] + tmpQ2[50]*tmpFx[33] + tmpQ2[51]*tmpFx[38] + tmpQ2[52]*tmpFx[43] + tmpQ2[53]*tmpFx[48] + tmpQ2[54]*tmpFx[53];
tmpQ1[24] = + tmpQ2[44]*tmpFx[4] + tmpQ2[45]*tmpFx[9] + tmpQ2[46]*tmpFx[14] + tmpQ2[47]*tmpFx[19] + tmpQ2[48]*tmpFx[24] + tmpQ2[49]*tmpFx[29] + tmpQ2[50]*tmpFx[34] + tmpQ2[51]*tmpFx[39] + tmpQ2[52]*tmpFx[44] + tmpQ2[53]*tmpFx[49] + tmpQ2[54]*tmpFx[54];
}

void acado_setObjR1R2( real_t* const tmpFu, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = + tmpFu[0];
tmpR2[1] = + tmpFu[2];
tmpR2[2] = + tmpFu[4];
tmpR2[3] = + tmpFu[6];
tmpR2[4] = + tmpFu[8];
tmpR2[5] = + tmpFu[10];
tmpR2[6] = + tmpFu[12];
tmpR2[7] = + tmpFu[14];
tmpR2[8] = + tmpFu[16];
tmpR2[9] = + tmpFu[18];
tmpR2[10] = + tmpFu[20];
tmpR2[11] = + tmpFu[1];
tmpR2[12] = + tmpFu[3];
tmpR2[13] = + tmpFu[5];
tmpR2[14] = + tmpFu[7];
tmpR2[15] = + tmpFu[9];
tmpR2[16] = + tmpFu[11];
tmpR2[17] = + tmpFu[13];
tmpR2[18] = + tmpFu[15];
tmpR2[19] = + tmpFu[17];
tmpR2[20] = + tmpFu[19];
tmpR2[21] = + tmpFu[21];
tmpR1[0] = + tmpR2[0]*tmpFu[0] + tmpR2[1]*tmpFu[2] + tmpR2[2]*tmpFu[4] + tmpR2[3]*tmpFu[6] + tmpR2[4]*tmpFu[8] + tmpR2[5]*tmpFu[10] + tmpR2[6]*tmpFu[12] + tmpR2[7]*tmpFu[14] + tmpR2[8]*tmpFu[16] + tmpR2[9]*tmpFu[18] + tmpR2[10]*tmpFu[20];
tmpR1[1] = + tmpR2[0]*tmpFu[1] + tmpR2[1]*tmpFu[3] + tmpR2[2]*tmpFu[5] + tmpR2[3]*tmpFu[7] + tmpR2[4]*tmpFu[9] + tmpR2[5]*tmpFu[11] + tmpR2[6]*tmpFu[13] + tmpR2[7]*tmpFu[15] + tmpR2[8]*tmpFu[17] + tmpR2[9]*tmpFu[19] + tmpR2[10]*tmpFu[21];
tmpR1[2] = + tmpR2[11]*tmpFu[0] + tmpR2[12]*tmpFu[2] + tmpR2[13]*tmpFu[4] + tmpR2[14]*tmpFu[6] + tmpR2[15]*tmpFu[8] + tmpR2[16]*tmpFu[10] + tmpR2[17]*tmpFu[12] + tmpR2[18]*tmpFu[14] + tmpR2[19]*tmpFu[16] + tmpR2[20]*tmpFu[18] + tmpR2[21]*tmpFu[20];
tmpR1[3] = + tmpR2[11]*tmpFu[1] + tmpR2[12]*tmpFu[3] + tmpR2[13]*tmpFu[5] + tmpR2[14]*tmpFu[7] + tmpR2[15]*tmpFu[9] + tmpR2[16]*tmpFu[11] + tmpR2[17]*tmpFu[13] + tmpR2[18]*tmpFu[15] + tmpR2[19]*tmpFu[17] + tmpR2[20]*tmpFu[19] + tmpR2[21]*tmpFu[21];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = + tmpFx[0]*(real_t)1.0000000000000000e+01;
tmpQN2[1] = + tmpFx[5]*(real_t)1.0000000000000000e+01;
tmpQN2[2] = + tmpFx[10]*(real_t)1.0000000000000000e+01;
tmpQN2[3] = + tmpFx[15]*(real_t)1.0000000000000000e+01;
tmpQN2[4] = + tmpFx[20]*(real_t)1.0000000000000000e+01;
tmpQN2[5] = + tmpFx[25]*(real_t)1.0000000000000000e+01;
tmpQN2[6] = + tmpFx[30]*(real_t)1.0000000000000000e+01;
tmpQN2[7] = + tmpFx[1]*(real_t)1.0000000000000000e+01;
tmpQN2[8] = + tmpFx[6]*(real_t)1.0000000000000000e+01;
tmpQN2[9] = + tmpFx[11]*(real_t)1.0000000000000000e+01;
tmpQN2[10] = + tmpFx[16]*(real_t)1.0000000000000000e+01;
tmpQN2[11] = + tmpFx[21]*(real_t)1.0000000000000000e+01;
tmpQN2[12] = + tmpFx[26]*(real_t)1.0000000000000000e+01;
tmpQN2[13] = + tmpFx[31]*(real_t)1.0000000000000000e+01;
tmpQN2[14] = + tmpFx[2]*(real_t)1.0000000000000000e+01;
tmpQN2[15] = + tmpFx[7]*(real_t)1.0000000000000000e+01;
tmpQN2[16] = + tmpFx[12]*(real_t)1.0000000000000000e+01;
tmpQN2[17] = + tmpFx[17]*(real_t)1.0000000000000000e+01;
tmpQN2[18] = + tmpFx[22]*(real_t)1.0000000000000000e+01;
tmpQN2[19] = + tmpFx[27]*(real_t)1.0000000000000000e+01;
tmpQN2[20] = + tmpFx[32]*(real_t)1.0000000000000000e+01;
tmpQN2[21] = + tmpFx[3]*(real_t)1.0000000000000000e+01;
tmpQN2[22] = + tmpFx[8]*(real_t)1.0000000000000000e+01;
tmpQN2[23] = + tmpFx[13]*(real_t)1.0000000000000000e+01;
tmpQN2[24] = + tmpFx[18]*(real_t)1.0000000000000000e+01;
tmpQN2[25] = + tmpFx[23]*(real_t)1.0000000000000000e+01;
tmpQN2[26] = + tmpFx[28]*(real_t)1.0000000000000000e+01;
tmpQN2[27] = + tmpFx[33]*(real_t)1.0000000000000000e+01;
tmpQN2[28] = + tmpFx[4]*(real_t)1.0000000000000000e+01;
tmpQN2[29] = + tmpFx[9]*(real_t)1.0000000000000000e+01;
tmpQN2[30] = + tmpFx[14]*(real_t)1.0000000000000000e+01;
tmpQN2[31] = + tmpFx[19]*(real_t)1.0000000000000000e+01;
tmpQN2[32] = + tmpFx[24]*(real_t)1.0000000000000000e+01;
tmpQN2[33] = + tmpFx[29]*(real_t)1.0000000000000000e+01;
tmpQN2[34] = + tmpFx[34]*(real_t)1.0000000000000000e+01;
tmpQN1[0] = + tmpQN2[0]*tmpFx[0] + tmpQN2[1]*tmpFx[5] + tmpQN2[2]*tmpFx[10] + tmpQN2[3]*tmpFx[15] + tmpQN2[4]*tmpFx[20] + tmpQN2[5]*tmpFx[25] + tmpQN2[6]*tmpFx[30];
tmpQN1[1] = + tmpQN2[0]*tmpFx[1] + tmpQN2[1]*tmpFx[6] + tmpQN2[2]*tmpFx[11] + tmpQN2[3]*tmpFx[16] + tmpQN2[4]*tmpFx[21] + tmpQN2[5]*tmpFx[26] + tmpQN2[6]*tmpFx[31];
tmpQN1[2] = + tmpQN2[0]*tmpFx[2] + tmpQN2[1]*tmpFx[7] + tmpQN2[2]*tmpFx[12] + tmpQN2[3]*tmpFx[17] + tmpQN2[4]*tmpFx[22] + tmpQN2[5]*tmpFx[27] + tmpQN2[6]*tmpFx[32];
tmpQN1[3] = + tmpQN2[0]*tmpFx[3] + tmpQN2[1]*tmpFx[8] + tmpQN2[2]*tmpFx[13] + tmpQN2[3]*tmpFx[18] + tmpQN2[4]*tmpFx[23] + tmpQN2[5]*tmpFx[28] + tmpQN2[6]*tmpFx[33];
tmpQN1[4] = + tmpQN2[0]*tmpFx[4] + tmpQN2[1]*tmpFx[9] + tmpQN2[2]*tmpFx[14] + tmpQN2[3]*tmpFx[19] + tmpQN2[4]*tmpFx[24] + tmpQN2[5]*tmpFx[29] + tmpQN2[6]*tmpFx[34];
tmpQN1[5] = + tmpQN2[7]*tmpFx[0] + tmpQN2[8]*tmpFx[5] + tmpQN2[9]*tmpFx[10] + tmpQN2[10]*tmpFx[15] + tmpQN2[11]*tmpFx[20] + tmpQN2[12]*tmpFx[25] + tmpQN2[13]*tmpFx[30];
tmpQN1[6] = + tmpQN2[7]*tmpFx[1] + tmpQN2[8]*tmpFx[6] + tmpQN2[9]*tmpFx[11] + tmpQN2[10]*tmpFx[16] + tmpQN2[11]*tmpFx[21] + tmpQN2[12]*tmpFx[26] + tmpQN2[13]*tmpFx[31];
tmpQN1[7] = + tmpQN2[7]*tmpFx[2] + tmpQN2[8]*tmpFx[7] + tmpQN2[9]*tmpFx[12] + tmpQN2[10]*tmpFx[17] + tmpQN2[11]*tmpFx[22] + tmpQN2[12]*tmpFx[27] + tmpQN2[13]*tmpFx[32];
tmpQN1[8] = + tmpQN2[7]*tmpFx[3] + tmpQN2[8]*tmpFx[8] + tmpQN2[9]*tmpFx[13] + tmpQN2[10]*tmpFx[18] + tmpQN2[11]*tmpFx[23] + tmpQN2[12]*tmpFx[28] + tmpQN2[13]*tmpFx[33];
tmpQN1[9] = + tmpQN2[7]*tmpFx[4] + tmpQN2[8]*tmpFx[9] + tmpQN2[9]*tmpFx[14] + tmpQN2[10]*tmpFx[19] + tmpQN2[11]*tmpFx[24] + tmpQN2[12]*tmpFx[29] + tmpQN2[13]*tmpFx[34];
tmpQN1[10] = + tmpQN2[14]*tmpFx[0] + tmpQN2[15]*tmpFx[5] + tmpQN2[16]*tmpFx[10] + tmpQN2[17]*tmpFx[15] + tmpQN2[18]*tmpFx[20] + tmpQN2[19]*tmpFx[25] + tmpQN2[20]*tmpFx[30];
tmpQN1[11] = + tmpQN2[14]*tmpFx[1] + tmpQN2[15]*tmpFx[6] + tmpQN2[16]*tmpFx[11] + tmpQN2[17]*tmpFx[16] + tmpQN2[18]*tmpFx[21] + tmpQN2[19]*tmpFx[26] + tmpQN2[20]*tmpFx[31];
tmpQN1[12] = + tmpQN2[14]*tmpFx[2] + tmpQN2[15]*tmpFx[7] + tmpQN2[16]*tmpFx[12] + tmpQN2[17]*tmpFx[17] + tmpQN2[18]*tmpFx[22] + tmpQN2[19]*tmpFx[27] + tmpQN2[20]*tmpFx[32];
tmpQN1[13] = + tmpQN2[14]*tmpFx[3] + tmpQN2[15]*tmpFx[8] + tmpQN2[16]*tmpFx[13] + tmpQN2[17]*tmpFx[18] + tmpQN2[18]*tmpFx[23] + tmpQN2[19]*tmpFx[28] + tmpQN2[20]*tmpFx[33];
tmpQN1[14] = + tmpQN2[14]*tmpFx[4] + tmpQN2[15]*tmpFx[9] + tmpQN2[16]*tmpFx[14] + tmpQN2[17]*tmpFx[19] + tmpQN2[18]*tmpFx[24] + tmpQN2[19]*tmpFx[29] + tmpQN2[20]*tmpFx[34];
tmpQN1[15] = + tmpQN2[21]*tmpFx[0] + tmpQN2[22]*tmpFx[5] + tmpQN2[23]*tmpFx[10] + tmpQN2[24]*tmpFx[15] + tmpQN2[25]*tmpFx[20] + tmpQN2[26]*tmpFx[25] + tmpQN2[27]*tmpFx[30];
tmpQN1[16] = + tmpQN2[21]*tmpFx[1] + tmpQN2[22]*tmpFx[6] + tmpQN2[23]*tmpFx[11] + tmpQN2[24]*tmpFx[16] + tmpQN2[25]*tmpFx[21] + tmpQN2[26]*tmpFx[26] + tmpQN2[27]*tmpFx[31];
tmpQN1[17] = + tmpQN2[21]*tmpFx[2] + tmpQN2[22]*tmpFx[7] + tmpQN2[23]*tmpFx[12] + tmpQN2[24]*tmpFx[17] + tmpQN2[25]*tmpFx[22] + tmpQN2[26]*tmpFx[27] + tmpQN2[27]*tmpFx[32];
tmpQN1[18] = + tmpQN2[21]*tmpFx[3] + tmpQN2[22]*tmpFx[8] + tmpQN2[23]*tmpFx[13] + tmpQN2[24]*tmpFx[18] + tmpQN2[25]*tmpFx[23] + tmpQN2[26]*tmpFx[28] + tmpQN2[27]*tmpFx[33];
tmpQN1[19] = + tmpQN2[21]*tmpFx[4] + tmpQN2[22]*tmpFx[9] + tmpQN2[23]*tmpFx[14] + tmpQN2[24]*tmpFx[19] + tmpQN2[25]*tmpFx[24] + tmpQN2[26]*tmpFx[29] + tmpQN2[27]*tmpFx[34];
tmpQN1[20] = + tmpQN2[28]*tmpFx[0] + tmpQN2[29]*tmpFx[5] + tmpQN2[30]*tmpFx[10] + tmpQN2[31]*tmpFx[15] + tmpQN2[32]*tmpFx[20] + tmpQN2[33]*tmpFx[25] + tmpQN2[34]*tmpFx[30];
tmpQN1[21] = + tmpQN2[28]*tmpFx[1] + tmpQN2[29]*tmpFx[6] + tmpQN2[30]*tmpFx[11] + tmpQN2[31]*tmpFx[16] + tmpQN2[32]*tmpFx[21] + tmpQN2[33]*tmpFx[26] + tmpQN2[34]*tmpFx[31];
tmpQN1[22] = + tmpQN2[28]*tmpFx[2] + tmpQN2[29]*tmpFx[7] + tmpQN2[30]*tmpFx[12] + tmpQN2[31]*tmpFx[17] + tmpQN2[32]*tmpFx[22] + tmpQN2[33]*tmpFx[27] + tmpQN2[34]*tmpFx[32];
tmpQN1[23] = + tmpQN2[28]*tmpFx[3] + tmpQN2[29]*tmpFx[8] + tmpQN2[30]*tmpFx[13] + tmpQN2[31]*tmpFx[18] + tmpQN2[32]*tmpFx[23] + tmpQN2[33]*tmpFx[28] + tmpQN2[34]*tmpFx[33];
tmpQN1[24] = + tmpQN2[28]*tmpFx[4] + tmpQN2[29]*tmpFx[9] + tmpQN2[30]*tmpFx[14] + tmpQN2[31]*tmpFx[19] + tmpQN2[32]*tmpFx[24] + tmpQN2[33]*tmpFx[29] + tmpQN2[34]*tmpFx[34];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[runObj * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[runObj * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 11] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 11 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 11 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 11 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 11 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 11 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 11 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 11 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 11 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 11 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 11 + 10] = acadoWorkspace.objValueOut[10];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 11 ]), &(acadoWorkspace.Q1[ runObj * 25 ]), &(acadoWorkspace.Q2[ runObj * 55 ]) );

acado_setObjR1R2( &(acadoWorkspace.objValueOut[ 66 ]), &(acadoWorkspace.R1[ runObj * 4 ]), &(acadoWorkspace.R2[ runObj * 22 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 7 ]), acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[5] + Gx1[2]*Gx2[10] + Gx1[3]*Gx2[15] + Gx1[4]*Gx2[20];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[6] + Gx1[2]*Gx2[11] + Gx1[3]*Gx2[16] + Gx1[4]*Gx2[21];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[7] + Gx1[2]*Gx2[12] + Gx1[3]*Gx2[17] + Gx1[4]*Gx2[22];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[13] + Gx1[3]*Gx2[18] + Gx1[4]*Gx2[23];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[14] + Gx1[3]*Gx2[19] + Gx1[4]*Gx2[24];
Gx3[5] = + Gx1[5]*Gx2[0] + Gx1[6]*Gx2[5] + Gx1[7]*Gx2[10] + Gx1[8]*Gx2[15] + Gx1[9]*Gx2[20];
Gx3[6] = + Gx1[5]*Gx2[1] + Gx1[6]*Gx2[6] + Gx1[7]*Gx2[11] + Gx1[8]*Gx2[16] + Gx1[9]*Gx2[21];
Gx3[7] = + Gx1[5]*Gx2[2] + Gx1[6]*Gx2[7] + Gx1[7]*Gx2[12] + Gx1[8]*Gx2[17] + Gx1[9]*Gx2[22];
Gx3[8] = + Gx1[5]*Gx2[3] + Gx1[6]*Gx2[8] + Gx1[7]*Gx2[13] + Gx1[8]*Gx2[18] + Gx1[9]*Gx2[23];
Gx3[9] = + Gx1[5]*Gx2[4] + Gx1[6]*Gx2[9] + Gx1[7]*Gx2[14] + Gx1[8]*Gx2[19] + Gx1[9]*Gx2[24];
Gx3[10] = + Gx1[10]*Gx2[0] + Gx1[11]*Gx2[5] + Gx1[12]*Gx2[10] + Gx1[13]*Gx2[15] + Gx1[14]*Gx2[20];
Gx3[11] = + Gx1[10]*Gx2[1] + Gx1[11]*Gx2[6] + Gx1[12]*Gx2[11] + Gx1[13]*Gx2[16] + Gx1[14]*Gx2[21];
Gx3[12] = + Gx1[10]*Gx2[2] + Gx1[11]*Gx2[7] + Gx1[12]*Gx2[12] + Gx1[13]*Gx2[17] + Gx1[14]*Gx2[22];
Gx3[13] = + Gx1[10]*Gx2[3] + Gx1[11]*Gx2[8] + Gx1[12]*Gx2[13] + Gx1[13]*Gx2[18] + Gx1[14]*Gx2[23];
Gx3[14] = + Gx1[10]*Gx2[4] + Gx1[11]*Gx2[9] + Gx1[12]*Gx2[14] + Gx1[13]*Gx2[19] + Gx1[14]*Gx2[24];
Gx3[15] = + Gx1[15]*Gx2[0] + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[15] + Gx1[19]*Gx2[20];
Gx3[16] = + Gx1[15]*Gx2[1] + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[21];
Gx3[17] = + Gx1[15]*Gx2[2] + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[22];
Gx3[18] = + Gx1[15]*Gx2[3] + Gx1[16]*Gx2[8] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[23];
Gx3[19] = + Gx1[15]*Gx2[4] + Gx1[16]*Gx2[9] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[24];
Gx3[20] = + Gx1[20]*Gx2[0] + Gx1[21]*Gx2[5] + Gx1[22]*Gx2[10] + Gx1[23]*Gx2[15] + Gx1[24]*Gx2[20];
Gx3[21] = + Gx1[20]*Gx2[1] + Gx1[21]*Gx2[6] + Gx1[22]*Gx2[11] + Gx1[23]*Gx2[16] + Gx1[24]*Gx2[21];
Gx3[22] = + Gx1[20]*Gx2[2] + Gx1[21]*Gx2[7] + Gx1[22]*Gx2[12] + Gx1[23]*Gx2[17] + Gx1[24]*Gx2[22];
Gx3[23] = + Gx1[20]*Gx2[3] + Gx1[21]*Gx2[8] + Gx1[22]*Gx2[13] + Gx1[23]*Gx2[18] + Gx1[24]*Gx2[23];
Gx3[24] = + Gx1[20]*Gx2[4] + Gx1[21]*Gx2[9] + Gx1[22]*Gx2[14] + Gx1[23]*Gx2[19] + Gx1[24]*Gx2[24];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[2] + Gx1[2]*Gu1[4] + Gx1[3]*Gu1[6] + Gx1[4]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[5] + Gx1[3]*Gu1[7] + Gx1[4]*Gu1[9];
Gu2[2] = + Gx1[5]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[7]*Gu1[4] + Gx1[8]*Gu1[6] + Gx1[9]*Gu1[8];
Gu2[3] = + Gx1[5]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[7]*Gu1[5] + Gx1[8]*Gu1[7] + Gx1[9]*Gu1[9];
Gu2[4] = + Gx1[10]*Gu1[0] + Gx1[11]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[13]*Gu1[6] + Gx1[14]*Gu1[8];
Gu2[5] = + Gx1[10]*Gu1[1] + Gx1[11]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[13]*Gu1[7] + Gx1[14]*Gu1[9];
Gu2[6] = + Gx1[15]*Gu1[0] + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[19]*Gu1[8];
Gu2[7] = + Gx1[15]*Gu1[1] + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[19]*Gu1[9];
Gu2[8] = + Gx1[20]*Gu1[0] + Gx1[21]*Gu1[2] + Gx1[22]*Gu1[4] + Gx1[23]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[20]*Gu1[1] + Gx1[21]*Gu1[3] + Gx1[22]*Gu1[5] + Gx1[23]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 42] = + Gu1[0]*Gu2[0] + Gu1[2]*Gu2[2] + Gu1[4]*Gu2[4] + Gu1[6]*Gu2[6] + Gu1[8]*Gu2[8] + R11[0];
acadoWorkspace.H[iRow * 42 + 1] = + Gu1[0]*Gu2[1] + Gu1[2]*Gu2[3] + Gu1[4]*Gu2[5] + Gu1[6]*Gu2[7] + Gu1[8]*Gu2[9] + R11[1];
acadoWorkspace.H[iRow * 42 + 20] = + Gu1[1]*Gu2[0] + Gu1[3]*Gu2[2] + Gu1[5]*Gu2[4] + Gu1[7]*Gu2[6] + Gu1[9]*Gu2[8] + R11[2];
acadoWorkspace.H[iRow * 42 + 21] = + Gu1[1]*Gu2[1] + Gu1[3]*Gu2[3] + Gu1[5]*Gu2[5] + Gu1[7]*Gu2[7] + Gu1[9]*Gu2[9] + R11[3];
acadoWorkspace.H[iRow * 42] += 1.0000000000000000e-10;
acadoWorkspace.H[iRow * 42 + 21] += 1.0000000000000000e-10;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[5]*Gu1[2] + Gx1[10]*Gu1[4] + Gx1[15]*Gu1[6] + Gx1[20]*Gu1[8];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[5]*Gu1[3] + Gx1[10]*Gu1[5] + Gx1[15]*Gu1[7] + Gx1[20]*Gu1[9];
Gu2[2] = + Gx1[1]*Gu1[0] + Gx1[6]*Gu1[2] + Gx1[11]*Gu1[4] + Gx1[16]*Gu1[6] + Gx1[21]*Gu1[8];
Gu2[3] = + Gx1[1]*Gu1[1] + Gx1[6]*Gu1[3] + Gx1[11]*Gu1[5] + Gx1[16]*Gu1[7] + Gx1[21]*Gu1[9];
Gu2[4] = + Gx1[2]*Gu1[0] + Gx1[7]*Gu1[2] + Gx1[12]*Gu1[4] + Gx1[17]*Gu1[6] + Gx1[22]*Gu1[8];
Gu2[5] = + Gx1[2]*Gu1[1] + Gx1[7]*Gu1[3] + Gx1[12]*Gu1[5] + Gx1[17]*Gu1[7] + Gx1[22]*Gu1[9];
Gu2[6] = + Gx1[3]*Gu1[0] + Gx1[8]*Gu1[2] + Gx1[13]*Gu1[4] + Gx1[18]*Gu1[6] + Gx1[23]*Gu1[8];
Gu2[7] = + Gx1[3]*Gu1[1] + Gx1[8]*Gu1[3] + Gx1[13]*Gu1[5] + Gx1[18]*Gu1[7] + Gx1[23]*Gu1[9];
Gu2[8] = + Gx1[4]*Gu1[0] + Gx1[9]*Gu1[2] + Gx1[14]*Gu1[4] + Gx1[19]*Gu1[6] + Gx1[24]*Gu1[8];
Gu2[9] = + Gx1[4]*Gu1[1] + Gx1[9]*Gu1[3] + Gx1[14]*Gu1[5] + Gx1[19]*Gu1[7] + Gx1[24]*Gu1[9];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[2] + Q11[2]*Gu1[4] + Q11[3]*Gu1[6] + Q11[4]*Gu1[8] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[3] + Q11[2]*Gu1[5] + Q11[3]*Gu1[7] + Q11[4]*Gu1[9] + Gu2[1];
Gu3[2] = + Q11[5]*Gu1[0] + Q11[6]*Gu1[2] + Q11[7]*Gu1[4] + Q11[8]*Gu1[6] + Q11[9]*Gu1[8] + Gu2[2];
Gu3[3] = + Q11[5]*Gu1[1] + Q11[6]*Gu1[3] + Q11[7]*Gu1[5] + Q11[8]*Gu1[7] + Q11[9]*Gu1[9] + Gu2[3];
Gu3[4] = + Q11[10]*Gu1[0] + Q11[11]*Gu1[2] + Q11[12]*Gu1[4] + Q11[13]*Gu1[6] + Q11[14]*Gu1[8] + Gu2[4];
Gu3[5] = + Q11[10]*Gu1[1] + Q11[11]*Gu1[3] + Q11[12]*Gu1[5] + Q11[13]*Gu1[7] + Q11[14]*Gu1[9] + Gu2[5];
Gu3[6] = + Q11[15]*Gu1[0] + Q11[16]*Gu1[2] + Q11[17]*Gu1[4] + Q11[18]*Gu1[6] + Q11[19]*Gu1[8] + Gu2[6];
Gu3[7] = + Q11[15]*Gu1[1] + Q11[16]*Gu1[3] + Q11[17]*Gu1[5] + Q11[18]*Gu1[7] + Q11[19]*Gu1[9] + Gu2[7];
Gu3[8] = + Q11[20]*Gu1[0] + Q11[21]*Gu1[2] + Q11[22]*Gu1[4] + Q11[23]*Gu1[6] + Q11[24]*Gu1[8] + Gu2[8];
Gu3[9] = + Q11[20]*Gu1[1] + Q11[21]*Gu1[3] + Q11[22]*Gu1[5] + Q11[23]*Gu1[7] + Q11[24]*Gu1[9] + Gu2[9];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[5]*w11[1] + Gx1[10]*w11[2] + Gx1[15]*w11[3] + Gx1[20]*w11[4] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[6]*w11[1] + Gx1[11]*w11[2] + Gx1[16]*w11[3] + Gx1[21]*w11[4] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[7]*w11[1] + Gx1[12]*w11[2] + Gx1[17]*w11[3] + Gx1[22]*w11[4] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[8]*w11[1] + Gx1[13]*w11[2] + Gx1[18]*w11[3] + Gx1[23]*w11[4] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[9]*w11[1] + Gx1[14]*w11[2] + Gx1[19]*w11[3] + Gx1[24]*w11[4] + w12[4];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[2]*w11[1] + Gu1[4]*w11[2] + Gu1[6]*w11[3] + Gu1[8]*w11[4];
U1[1] += + Gu1[1]*w11[0] + Gu1[3]*w11[1] + Gu1[5]*w11[2] + Gu1[7]*w11[3] + Gu1[9]*w11[4];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + w12[0];
w13[1] = + Q11[5]*w11[0] + Q11[6]*w11[1] + Q11[7]*w11[2] + Q11[8]*w11[3] + Q11[9]*w11[4] + w12[1];
w13[2] = + Q11[10]*w11[0] + Q11[11]*w11[1] + Q11[12]*w11[2] + Q11[13]*w11[3] + Q11[14]*w11[4] + w12[2];
w13[3] = + Q11[15]*w11[0] + Q11[16]*w11[1] + Q11[17]*w11[2] + Q11[18]*w11[3] + Q11[19]*w11[4] + w12[3];
w13[4] = + Q11[20]*w11[0] + Q11[21]*w11[1] + Q11[22]*w11[2] + Q11[23]*w11[3] + Q11[24]*w11[4] + w12[4];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4];
w12[1] += + Gx1[5]*w11[0] + Gx1[6]*w11[1] + Gx1[7]*w11[2] + Gx1[8]*w11[3] + Gx1[9]*w11[4];
w12[2] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4];
w12[3] += + Gx1[15]*w11[0] + Gx1[16]*w11[1] + Gx1[17]*w11[2] + Gx1[18]*w11[3] + Gx1[19]*w11[4];
w12[4] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4];
w12[1] += + Gx1[5]*w11[0] + Gx1[6]*w11[1] + Gx1[7]*w11[2] + Gx1[8]*w11[3] + Gx1[9]*w11[4];
w12[2] += + Gx1[10]*w11[0] + Gx1[11]*w11[1] + Gx1[12]*w11[2] + Gx1[13]*w11[3] + Gx1[14]*w11[4];
w12[3] += + Gx1[15]*w11[0] + Gx1[16]*w11[1] + Gx1[17]*w11[2] + Gx1[18]*w11[3] + Gx1[19]*w11[4];
w12[4] += + Gx1[20]*w11[0] + Gx1[21]*w11[1] + Gx1[22]*w11[2] + Gx1[23]*w11[3] + Gx1[24]*w11[4];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1];
w12[1] += + Gu1[2]*U1[0] + Gu1[3]*U1[1];
w12[2] += + Gu1[4]*U1[0] + Gu1[5]*U1[1];
w12[3] += + Gu1[6]*U1[0] + Gu1[7]*U1[1];
w12[4] += + Gu1[8]*U1[0] + Gu1[9]*U1[1];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 40) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2)] = acadoWorkspace.H[(iCol * 40) + (iRow * 2 + 1)];
acadoWorkspace.H[(iRow * 40 + 20) + (iCol * 2 + 1)] = acadoWorkspace.H[(iCol * 40 + 20) + (iRow * 2 + 1)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10];
RDy1[1] = + R2[11]*Dy1[0] + R2[12]*Dy1[1] + R2[13]*Dy1[2] + R2[14]*Dy1[3] + R2[15]*Dy1[4] + R2[16]*Dy1[5] + R2[17]*Dy1[6] + R2[18]*Dy1[7] + R2[19]*Dy1[8] + R2[20]*Dy1[9] + R2[21]*Dy1[10];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10];
QDy1[1] = + Q2[11]*Dy1[0] + Q2[12]*Dy1[1] + Q2[13]*Dy1[2] + Q2[14]*Dy1[3] + Q2[15]*Dy1[4] + Q2[16]*Dy1[5] + Q2[17]*Dy1[6] + Q2[18]*Dy1[7] + Q2[19]*Dy1[8] + Q2[20]*Dy1[9] + Q2[21]*Dy1[10];
QDy1[2] = + Q2[22]*Dy1[0] + Q2[23]*Dy1[1] + Q2[24]*Dy1[2] + Q2[25]*Dy1[3] + Q2[26]*Dy1[4] + Q2[27]*Dy1[5] + Q2[28]*Dy1[6] + Q2[29]*Dy1[7] + Q2[30]*Dy1[8] + Q2[31]*Dy1[9] + Q2[32]*Dy1[10];
QDy1[3] = + Q2[33]*Dy1[0] + Q2[34]*Dy1[1] + Q2[35]*Dy1[2] + Q2[36]*Dy1[3] + Q2[37]*Dy1[4] + Q2[38]*Dy1[5] + Q2[39]*Dy1[6] + Q2[40]*Dy1[7] + Q2[41]*Dy1[8] + Q2[42]*Dy1[9] + Q2[43]*Dy1[10];
QDy1[4] = + Q2[44]*Dy1[0] + Q2[45]*Dy1[1] + Q2[46]*Dy1[2] + Q2[47]*Dy1[3] + Q2[48]*Dy1[4] + Q2[49]*Dy1[5] + Q2[50]*Dy1[6] + Q2[51]*Dy1[7] + Q2[52]*Dy1[8] + Q2[53]*Dy1[9] + Q2[54]*Dy1[10];
}

void acado_multHxE( real_t* const Hx, real_t* const E, int row, int col )
{
acadoWorkspace.A[(row * 120 + 200) + (col * 2)] = + Hx[0]*E[0] + Hx[1]*E[2] + Hx[2]*E[4] + Hx[3]*E[6] + Hx[4]*E[8];
acadoWorkspace.A[(row * 120 + 200) + (col * 2 + 1)] = + Hx[0]*E[1] + Hx[1]*E[3] + Hx[2]*E[5] + Hx[3]*E[7] + Hx[4]*E[9];
acadoWorkspace.A[(row * 120 + 220) + (col * 2)] = + Hx[5]*E[0] + Hx[6]*E[2] + Hx[7]*E[4] + Hx[8]*E[6] + Hx[9]*E[8];
acadoWorkspace.A[(row * 120 + 220) + (col * 2 + 1)] = + Hx[5]*E[1] + Hx[6]*E[3] + Hx[7]*E[5] + Hx[8]*E[7] + Hx[9]*E[9];
acadoWorkspace.A[(row * 120 + 240) + (col * 2)] = + Hx[10]*E[0] + Hx[11]*E[2] + Hx[12]*E[4] + Hx[13]*E[6] + Hx[14]*E[8];
acadoWorkspace.A[(row * 120 + 240) + (col * 2 + 1)] = + Hx[10]*E[1] + Hx[11]*E[3] + Hx[12]*E[5] + Hx[13]*E[7] + Hx[14]*E[9];
acadoWorkspace.A[(row * 120 + 260) + (col * 2)] = + Hx[15]*E[0] + Hx[16]*E[2] + Hx[17]*E[4] + Hx[18]*E[6] + Hx[19]*E[8];
acadoWorkspace.A[(row * 120 + 260) + (col * 2 + 1)] = + Hx[15]*E[1] + Hx[16]*E[3] + Hx[17]*E[5] + Hx[18]*E[7] + Hx[19]*E[9];
acadoWorkspace.A[(row * 120 + 280) + (col * 2)] = + Hx[20]*E[0] + Hx[21]*E[2] + Hx[22]*E[4] + Hx[23]*E[6] + Hx[24]*E[8];
acadoWorkspace.A[(row * 120 + 280) + (col * 2 + 1)] = + Hx[20]*E[1] + Hx[21]*E[3] + Hx[22]*E[5] + Hx[23]*E[7] + Hx[24]*E[9];
acadoWorkspace.A[(row * 120 + 300) + (col * 2)] = + Hx[25]*E[0] + Hx[26]*E[2] + Hx[27]*E[4] + Hx[28]*E[6] + Hx[29]*E[8];
acadoWorkspace.A[(row * 120 + 300) + (col * 2 + 1)] = + Hx[25]*E[1] + Hx[26]*E[3] + Hx[27]*E[5] + Hx[28]*E[7] + Hx[29]*E[9];
}

void acado_macHxd( real_t* const Hx, real_t* const tmpd, real_t* const lbA, real_t* const ubA )
{
acadoWorkspace.evHxd[0] = + Hx[0]*tmpd[0] + Hx[1]*tmpd[1] + Hx[2]*tmpd[2] + Hx[3]*tmpd[3] + Hx[4]*tmpd[4];
acadoWorkspace.evHxd[1] = + Hx[5]*tmpd[0] + Hx[6]*tmpd[1] + Hx[7]*tmpd[2] + Hx[8]*tmpd[3] + Hx[9]*tmpd[4];
acadoWorkspace.evHxd[2] = + Hx[10]*tmpd[0] + Hx[11]*tmpd[1] + Hx[12]*tmpd[2] + Hx[13]*tmpd[3] + Hx[14]*tmpd[4];
acadoWorkspace.evHxd[3] = + Hx[15]*tmpd[0] + Hx[16]*tmpd[1] + Hx[17]*tmpd[2] + Hx[18]*tmpd[3] + Hx[19]*tmpd[4];
acadoWorkspace.evHxd[4] = + Hx[20]*tmpd[0] + Hx[21]*tmpd[1] + Hx[22]*tmpd[2] + Hx[23]*tmpd[3] + Hx[24]*tmpd[4];
acadoWorkspace.evHxd[5] = + Hx[25]*tmpd[0] + Hx[26]*tmpd[1] + Hx[27]*tmpd[2] + Hx[28]*tmpd[3] + Hx[29]*tmpd[4];
lbA[0] -= acadoWorkspace.evHxd[0];
lbA[1] -= acadoWorkspace.evHxd[1];
lbA[2] -= acadoWorkspace.evHxd[2];
lbA[3] -= acadoWorkspace.evHxd[3];
lbA[4] -= acadoWorkspace.evHxd[4];
lbA[5] -= acadoWorkspace.evHxd[5];
ubA[0] -= acadoWorkspace.evHxd[0];
ubA[1] -= acadoWorkspace.evHxd[1];
ubA[2] -= acadoWorkspace.evHxd[2];
ubA[3] -= acadoWorkspace.evHxd[3];
ubA[4] -= acadoWorkspace.evHxd[4];
ubA[5] -= acadoWorkspace.evHxd[5];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 10 */
static const int xBoundIndices[ 10 ] = 
{ 9, 14, 19, 24, 29, 34, 39, 44, 49, 54 };
acado_moveGxT( acadoWorkspace.evGx, acadoWorkspace.C );
acado_multGxGx( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.C, &(acadoWorkspace.C[ 25 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.C[ 25 ]), &(acadoWorkspace.C[ 50 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.C[ 50 ]), &(acadoWorkspace.C[ 75 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.C[ 75 ]), &(acadoWorkspace.C[ 100 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.C[ 100 ]), &(acadoWorkspace.C[ 125 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.C[ 125 ]), &(acadoWorkspace.C[ 150 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.C[ 150 ]), &(acadoWorkspace.C[ 175 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.C[ 175 ]), &(acadoWorkspace.C[ 200 ]) );
acado_multGxGx( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.C[ 200 ]), &(acadoWorkspace.C[ 225 ]) );
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 10 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.E[ 10 ]), &(acadoWorkspace.E[ 20 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.E[ 20 ]), &(acadoWorkspace.E[ 30 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.E[ 30 ]), &(acadoWorkspace.E[ 40 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.E[ 40 ]), &(acadoWorkspace.E[ 50 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 50 ]), &(acadoWorkspace.E[ 60 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 60 ]), &(acadoWorkspace.E[ 70 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 70 ]), &(acadoWorkspace.E[ 80 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 80 ]), &(acadoWorkspace.E[ 90 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 90 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 80 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 70 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 60 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 50 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 40 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 30 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 20 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.E[ 10 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 25 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.E[ 100 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.E[ 100 ]), &(acadoWorkspace.E[ 110 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.E[ 110 ]), &(acadoWorkspace.E[ 120 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.E[ 120 ]), &(acadoWorkspace.E[ 130 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.E[ 130 ]), &(acadoWorkspace.E[ 140 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 140 ]), &(acadoWorkspace.E[ 150 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 150 ]), &(acadoWorkspace.E[ 160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 160 ]), &(acadoWorkspace.E[ 170 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 170 ]), &(acadoWorkspace.E[ 180 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 180 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 170 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 150 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 140 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 130 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 120 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 110 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.E[ 100 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 4 ]), &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.E[ 190 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.E[ 190 ]), &(acadoWorkspace.E[ 200 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.E[ 200 ]), &(acadoWorkspace.E[ 210 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.E[ 210 ]), &(acadoWorkspace.E[ 220 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 220 ]), &(acadoWorkspace.E[ 230 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 230 ]), &(acadoWorkspace.E[ 240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 240 ]), &(acadoWorkspace.E[ 250 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 250 ]), &(acadoWorkspace.E[ 260 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 260 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 250 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 230 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 220 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 210 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 200 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.E[ 190 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 8 ]), &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.E[ 270 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 280 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.E[ 280 ]), &(acadoWorkspace.E[ 290 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 290 ]), &(acadoWorkspace.E[ 300 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 300 ]), &(acadoWorkspace.E[ 310 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 310 ]), &(acadoWorkspace.E[ 320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 320 ]), &(acadoWorkspace.E[ 330 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 330 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 310 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 300 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 290 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 280 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.E[ 270 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 12 ]), &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.E[ 340 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.E[ 340 ]), &(acadoWorkspace.E[ 350 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 350 ]), &(acadoWorkspace.E[ 360 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 360 ]), &(acadoWorkspace.E[ 370 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 370 ]), &(acadoWorkspace.E[ 380 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 380 ]), &(acadoWorkspace.E[ 390 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 390 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 380 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 370 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 360 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 350 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.E[ 340 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 16 ]), &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 50 ]), &(acadoWorkspace.E[ 400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.E[ 400 ]), &(acadoWorkspace.E[ 410 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 410 ]), &(acadoWorkspace.E[ 420 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 420 ]), &(acadoWorkspace.E[ 430 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 430 ]), &(acadoWorkspace.E[ 440 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 440 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 430 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 420 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 410 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.E[ 400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 20 ]), &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.E[ 450 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.E[ 450 ]), &(acadoWorkspace.E[ 460 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 460 ]), &(acadoWorkspace.E[ 470 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 470 ]), &(acadoWorkspace.E[ 480 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 480 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 470 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 460 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.E[ 450 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 24 ]), &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.E[ 490 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.E[ 490 ]), &(acadoWorkspace.E[ 500 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 500 ]), &(acadoWorkspace.E[ 510 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 510 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 500 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.E[ 490 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 28 ]), &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.E[ 520 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.E[ 520 ]), &(acadoWorkspace.E[ 530 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 530 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.E[ 520 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 32 ]), &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.E[ 540 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 540 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 36 ]), &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

acadoWorkspace.sbar[5] = acadoWorkspace.d[0];
acadoWorkspace.sbar[6] = acadoWorkspace.d[1];
acadoWorkspace.sbar[7] = acadoWorkspace.d[2];
acadoWorkspace.sbar[8] = acadoWorkspace.d[3];
acadoWorkspace.sbar[9] = acadoWorkspace.d[4];
acadoWorkspace.sbar[10] = acadoWorkspace.d[5];
acadoWorkspace.sbar[11] = acadoWorkspace.d[6];
acadoWorkspace.sbar[12] = acadoWorkspace.d[7];
acadoWorkspace.sbar[13] = acadoWorkspace.d[8];
acadoWorkspace.sbar[14] = acadoWorkspace.d[9];
acadoWorkspace.sbar[15] = acadoWorkspace.d[10];
acadoWorkspace.sbar[16] = acadoWorkspace.d[11];
acadoWorkspace.sbar[17] = acadoWorkspace.d[12];
acadoWorkspace.sbar[18] = acadoWorkspace.d[13];
acadoWorkspace.sbar[19] = acadoWorkspace.d[14];
acadoWorkspace.sbar[20] = acadoWorkspace.d[15];
acadoWorkspace.sbar[21] = acadoWorkspace.d[16];
acadoWorkspace.sbar[22] = acadoWorkspace.d[17];
acadoWorkspace.sbar[23] = acadoWorkspace.d[18];
acadoWorkspace.sbar[24] = acadoWorkspace.d[19];
acadoWorkspace.sbar[25] = acadoWorkspace.d[20];
acadoWorkspace.sbar[26] = acadoWorkspace.d[21];
acadoWorkspace.sbar[27] = acadoWorkspace.d[22];
acadoWorkspace.sbar[28] = acadoWorkspace.d[23];
acadoWorkspace.sbar[29] = acadoWorkspace.d[24];
acadoWorkspace.sbar[30] = acadoWorkspace.d[25];
acadoWorkspace.sbar[31] = acadoWorkspace.d[26];
acadoWorkspace.sbar[32] = acadoWorkspace.d[27];
acadoWorkspace.sbar[33] = acadoWorkspace.d[28];
acadoWorkspace.sbar[34] = acadoWorkspace.d[29];
acadoWorkspace.sbar[35] = acadoWorkspace.d[30];
acadoWorkspace.sbar[36] = acadoWorkspace.d[31];
acadoWorkspace.sbar[37] = acadoWorkspace.d[32];
acadoWorkspace.sbar[38] = acadoWorkspace.d[33];
acadoWorkspace.sbar[39] = acadoWorkspace.d[34];
acadoWorkspace.sbar[40] = acadoWorkspace.d[35];
acadoWorkspace.sbar[41] = acadoWorkspace.d[36];
acadoWorkspace.sbar[42] = acadoWorkspace.d[37];
acadoWorkspace.sbar[43] = acadoWorkspace.d[38];
acadoWorkspace.sbar[44] = acadoWorkspace.d[39];
acadoWorkspace.sbar[45] = acadoWorkspace.d[40];
acadoWorkspace.sbar[46] = acadoWorkspace.d[41];
acadoWorkspace.sbar[47] = acadoWorkspace.d[42];
acadoWorkspace.sbar[48] = acadoWorkspace.d[43];
acadoWorkspace.sbar[49] = acadoWorkspace.d[44];
acadoWorkspace.sbar[50] = acadoWorkspace.d[45];
acadoWorkspace.sbar[51] = acadoWorkspace.d[46];
acadoWorkspace.sbar[52] = acadoWorkspace.d[47];
acadoWorkspace.sbar[53] = acadoWorkspace.d[48];
acadoWorkspace.sbar[54] = acadoWorkspace.d[49];
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+12 - acadoVariables.u[19];
acadoWorkspace.ub[0] = (real_t)2.0000000000000001e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+12 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)2.0000000000000001e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+12 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)2.0000000000000001e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+12 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)2.0000000000000001e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+12 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)2.0000000000000001e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+12 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)2.0000000000000001e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+12 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)2.0000000000000001e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+12 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)2.0000000000000001e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+12 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)2.0000000000000001e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+12 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)2.0000000000000001e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+12 - acadoVariables.u[19];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 5;
lRun4 = ((lRun3) / (5)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 19)) / (2)) + (lRun4)) - (1)) * (5)) + ((lRun3) % (5));
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2)] = acadoWorkspace.E[lRun5 * 2];
acadoWorkspace.A[(lRun1 * 20) + (lRun2 * 2 + 1)] = acadoWorkspace.E[lRun5 * 2 + 1];
}
}

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.conValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.conValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.conValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.conValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.conValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.conValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.conValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];
acado_evaluatePathConstraints( acadoWorkspace.conValueIn, acadoWorkspace.conValueOut );
acadoWorkspace.evH[lRun1 * 6] = acadoWorkspace.conValueOut[0];
acadoWorkspace.evH[lRun1 * 6 + 1] = acadoWorkspace.conValueOut[1];
acadoWorkspace.evH[lRun1 * 6 + 2] = acadoWorkspace.conValueOut[2];
acadoWorkspace.evH[lRun1 * 6 + 3] = acadoWorkspace.conValueOut[3];
acadoWorkspace.evH[lRun1 * 6 + 4] = acadoWorkspace.conValueOut[4];
acadoWorkspace.evH[lRun1 * 6 + 5] = acadoWorkspace.conValueOut[5];

acadoWorkspace.evHx[lRun1 * 30] = acadoWorkspace.conValueOut[6];
acadoWorkspace.evHx[lRun1 * 30 + 1] = acadoWorkspace.conValueOut[7];
acadoWorkspace.evHx[lRun1 * 30 + 2] = acadoWorkspace.conValueOut[8];
acadoWorkspace.evHx[lRun1 * 30 + 3] = acadoWorkspace.conValueOut[9];
acadoWorkspace.evHx[lRun1 * 30 + 4] = acadoWorkspace.conValueOut[10];
acadoWorkspace.evHx[lRun1 * 30 + 5] = acadoWorkspace.conValueOut[11];
acadoWorkspace.evHx[lRun1 * 30 + 6] = acadoWorkspace.conValueOut[12];
acadoWorkspace.evHx[lRun1 * 30 + 7] = acadoWorkspace.conValueOut[13];
acadoWorkspace.evHx[lRun1 * 30 + 8] = acadoWorkspace.conValueOut[14];
acadoWorkspace.evHx[lRun1 * 30 + 9] = acadoWorkspace.conValueOut[15];
acadoWorkspace.evHx[lRun1 * 30 + 10] = acadoWorkspace.conValueOut[16];
acadoWorkspace.evHx[lRun1 * 30 + 11] = acadoWorkspace.conValueOut[17];
acadoWorkspace.evHx[lRun1 * 30 + 12] = acadoWorkspace.conValueOut[18];
acadoWorkspace.evHx[lRun1 * 30 + 13] = acadoWorkspace.conValueOut[19];
acadoWorkspace.evHx[lRun1 * 30 + 14] = acadoWorkspace.conValueOut[20];
acadoWorkspace.evHx[lRun1 * 30 + 15] = acadoWorkspace.conValueOut[21];
acadoWorkspace.evHx[lRun1 * 30 + 16] = acadoWorkspace.conValueOut[22];
acadoWorkspace.evHx[lRun1 * 30 + 17] = acadoWorkspace.conValueOut[23];
acadoWorkspace.evHx[lRun1 * 30 + 18] = acadoWorkspace.conValueOut[24];
acadoWorkspace.evHx[lRun1 * 30 + 19] = acadoWorkspace.conValueOut[25];
acadoWorkspace.evHx[lRun1 * 30 + 20] = acadoWorkspace.conValueOut[26];
acadoWorkspace.evHx[lRun1 * 30 + 21] = acadoWorkspace.conValueOut[27];
acadoWorkspace.evHx[lRun1 * 30 + 22] = acadoWorkspace.conValueOut[28];
acadoWorkspace.evHx[lRun1 * 30 + 23] = acadoWorkspace.conValueOut[29];
acadoWorkspace.evHx[lRun1 * 30 + 24] = acadoWorkspace.conValueOut[30];
acadoWorkspace.evHx[lRun1 * 30 + 25] = acadoWorkspace.conValueOut[31];
acadoWorkspace.evHx[lRun1 * 30 + 26] = acadoWorkspace.conValueOut[32];
acadoWorkspace.evHx[lRun1 * 30 + 27] = acadoWorkspace.conValueOut[33];
acadoWorkspace.evHx[lRun1 * 30 + 28] = acadoWorkspace.conValueOut[34];
acadoWorkspace.evHx[lRun1 * 30 + 29] = acadoWorkspace.conValueOut[35];
acadoWorkspace.evHu[lRun1 * 12] = acadoWorkspace.conValueOut[36];
acadoWorkspace.evHu[lRun1 * 12 + 1] = acadoWorkspace.conValueOut[37];
acadoWorkspace.evHu[lRun1 * 12 + 2] = acadoWorkspace.conValueOut[38];
acadoWorkspace.evHu[lRun1 * 12 + 3] = acadoWorkspace.conValueOut[39];
acadoWorkspace.evHu[lRun1 * 12 + 4] = acadoWorkspace.conValueOut[40];
acadoWorkspace.evHu[lRun1 * 12 + 5] = acadoWorkspace.conValueOut[41];
acadoWorkspace.evHu[lRun1 * 12 + 6] = acadoWorkspace.conValueOut[42];
acadoWorkspace.evHu[lRun1 * 12 + 7] = acadoWorkspace.conValueOut[43];
acadoWorkspace.evHu[lRun1 * 12 + 8] = acadoWorkspace.conValueOut[44];
acadoWorkspace.evHu[lRun1 * 12 + 9] = acadoWorkspace.conValueOut[45];
acadoWorkspace.evHu[lRun1 * 12 + 10] = acadoWorkspace.conValueOut[46];
acadoWorkspace.evHu[lRun1 * 12 + 11] = acadoWorkspace.conValueOut[47];
}



acado_multHxE( &(acadoWorkspace.evHx[ 30 ]), acadoWorkspace.E, 1, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 10 ]), 2, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.E[ 100 ]), 2, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 20 ]), 3, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 110 ]), 3, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.E[ 190 ]), 3, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 30 ]), 4, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 120 ]), 4, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 200 ]), 4, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.E[ 270 ]), 4, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 40 ]), 5, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 130 ]), 5, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 210 ]), 5, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 280 ]), 5, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.E[ 340 ]), 5, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 50 ]), 6, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 140 ]), 6, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 220 ]), 6, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 290 ]), 6, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 350 ]), 6, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.E[ 400 ]), 6, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 60 ]), 7, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 150 ]), 7, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 230 ]), 7, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 300 ]), 7, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 360 ]), 7, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 410 ]), 7, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.E[ 450 ]), 7, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 70 ]), 8, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 160 ]), 8, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 240 ]), 8, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 310 ]), 8, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 370 ]), 8, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 420 ]), 8, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 460 ]), 8, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.E[ 490 ]), 8, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 80 ]), 9, 0 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 170 ]), 9, 1 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 250 ]), 9, 2 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 320 ]), 9, 3 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 380 ]), 9, 4 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 430 ]), 9, 5 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 470 ]), 9, 6 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 500 ]), 9, 7 );
acado_multHxE( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.E[ 520 ]), 9, 8 );

acadoWorkspace.A[200] = acadoWorkspace.evHu[0];
acadoWorkspace.A[201] = acadoWorkspace.evHu[1];
acadoWorkspace.A[220] = acadoWorkspace.evHu[2];
acadoWorkspace.A[221] = acadoWorkspace.evHu[3];
acadoWorkspace.A[240] = acadoWorkspace.evHu[4];
acadoWorkspace.A[241] = acadoWorkspace.evHu[5];
acadoWorkspace.A[260] = acadoWorkspace.evHu[6];
acadoWorkspace.A[261] = acadoWorkspace.evHu[7];
acadoWorkspace.A[280] = acadoWorkspace.evHu[8];
acadoWorkspace.A[281] = acadoWorkspace.evHu[9];
acadoWorkspace.A[300] = acadoWorkspace.evHu[10];
acadoWorkspace.A[301] = acadoWorkspace.evHu[11];
acadoWorkspace.A[322] = acadoWorkspace.evHu[12];
acadoWorkspace.A[323] = acadoWorkspace.evHu[13];
acadoWorkspace.A[342] = acadoWorkspace.evHu[14];
acadoWorkspace.A[343] = acadoWorkspace.evHu[15];
acadoWorkspace.A[362] = acadoWorkspace.evHu[16];
acadoWorkspace.A[363] = acadoWorkspace.evHu[17];
acadoWorkspace.A[382] = acadoWorkspace.evHu[18];
acadoWorkspace.A[383] = acadoWorkspace.evHu[19];
acadoWorkspace.A[402] = acadoWorkspace.evHu[20];
acadoWorkspace.A[403] = acadoWorkspace.evHu[21];
acadoWorkspace.A[422] = acadoWorkspace.evHu[22];
acadoWorkspace.A[423] = acadoWorkspace.evHu[23];
acadoWorkspace.A[444] = acadoWorkspace.evHu[24];
acadoWorkspace.A[445] = acadoWorkspace.evHu[25];
acadoWorkspace.A[464] = acadoWorkspace.evHu[26];
acadoWorkspace.A[465] = acadoWorkspace.evHu[27];
acadoWorkspace.A[484] = acadoWorkspace.evHu[28];
acadoWorkspace.A[485] = acadoWorkspace.evHu[29];
acadoWorkspace.A[504] = acadoWorkspace.evHu[30];
acadoWorkspace.A[505] = acadoWorkspace.evHu[31];
acadoWorkspace.A[524] = acadoWorkspace.evHu[32];
acadoWorkspace.A[525] = acadoWorkspace.evHu[33];
acadoWorkspace.A[544] = acadoWorkspace.evHu[34];
acadoWorkspace.A[545] = acadoWorkspace.evHu[35];
acadoWorkspace.A[566] = acadoWorkspace.evHu[36];
acadoWorkspace.A[567] = acadoWorkspace.evHu[37];
acadoWorkspace.A[586] = acadoWorkspace.evHu[38];
acadoWorkspace.A[587] = acadoWorkspace.evHu[39];
acadoWorkspace.A[606] = acadoWorkspace.evHu[40];
acadoWorkspace.A[607] = acadoWorkspace.evHu[41];
acadoWorkspace.A[626] = acadoWorkspace.evHu[42];
acadoWorkspace.A[627] = acadoWorkspace.evHu[43];
acadoWorkspace.A[646] = acadoWorkspace.evHu[44];
acadoWorkspace.A[647] = acadoWorkspace.evHu[45];
acadoWorkspace.A[666] = acadoWorkspace.evHu[46];
acadoWorkspace.A[667] = acadoWorkspace.evHu[47];
acadoWorkspace.A[688] = acadoWorkspace.evHu[48];
acadoWorkspace.A[689] = acadoWorkspace.evHu[49];
acadoWorkspace.A[708] = acadoWorkspace.evHu[50];
acadoWorkspace.A[709] = acadoWorkspace.evHu[51];
acadoWorkspace.A[728] = acadoWorkspace.evHu[52];
acadoWorkspace.A[729] = acadoWorkspace.evHu[53];
acadoWorkspace.A[748] = acadoWorkspace.evHu[54];
acadoWorkspace.A[749] = acadoWorkspace.evHu[55];
acadoWorkspace.A[768] = acadoWorkspace.evHu[56];
acadoWorkspace.A[769] = acadoWorkspace.evHu[57];
acadoWorkspace.A[788] = acadoWorkspace.evHu[58];
acadoWorkspace.A[789] = acadoWorkspace.evHu[59];
acadoWorkspace.A[810] = acadoWorkspace.evHu[60];
acadoWorkspace.A[811] = acadoWorkspace.evHu[61];
acadoWorkspace.A[830] = acadoWorkspace.evHu[62];
acadoWorkspace.A[831] = acadoWorkspace.evHu[63];
acadoWorkspace.A[850] = acadoWorkspace.evHu[64];
acadoWorkspace.A[851] = acadoWorkspace.evHu[65];
acadoWorkspace.A[870] = acadoWorkspace.evHu[66];
acadoWorkspace.A[871] = acadoWorkspace.evHu[67];
acadoWorkspace.A[890] = acadoWorkspace.evHu[68];
acadoWorkspace.A[891] = acadoWorkspace.evHu[69];
acadoWorkspace.A[910] = acadoWorkspace.evHu[70];
acadoWorkspace.A[911] = acadoWorkspace.evHu[71];
acadoWorkspace.A[932] = acadoWorkspace.evHu[72];
acadoWorkspace.A[933] = acadoWorkspace.evHu[73];
acadoWorkspace.A[952] = acadoWorkspace.evHu[74];
acadoWorkspace.A[953] = acadoWorkspace.evHu[75];
acadoWorkspace.A[972] = acadoWorkspace.evHu[76];
acadoWorkspace.A[973] = acadoWorkspace.evHu[77];
acadoWorkspace.A[992] = acadoWorkspace.evHu[78];
acadoWorkspace.A[993] = acadoWorkspace.evHu[79];
acadoWorkspace.A[1012] = acadoWorkspace.evHu[80];
acadoWorkspace.A[1013] = acadoWorkspace.evHu[81];
acadoWorkspace.A[1032] = acadoWorkspace.evHu[82];
acadoWorkspace.A[1033] = acadoWorkspace.evHu[83];
acadoWorkspace.A[1054] = acadoWorkspace.evHu[84];
acadoWorkspace.A[1055] = acadoWorkspace.evHu[85];
acadoWorkspace.A[1074] = acadoWorkspace.evHu[86];
acadoWorkspace.A[1075] = acadoWorkspace.evHu[87];
acadoWorkspace.A[1094] = acadoWorkspace.evHu[88];
acadoWorkspace.A[1095] = acadoWorkspace.evHu[89];
acadoWorkspace.A[1114] = acadoWorkspace.evHu[90];
acadoWorkspace.A[1115] = acadoWorkspace.evHu[91];
acadoWorkspace.A[1134] = acadoWorkspace.evHu[92];
acadoWorkspace.A[1135] = acadoWorkspace.evHu[93];
acadoWorkspace.A[1154] = acadoWorkspace.evHu[94];
acadoWorkspace.A[1155] = acadoWorkspace.evHu[95];
acadoWorkspace.A[1176] = acadoWorkspace.evHu[96];
acadoWorkspace.A[1177] = acadoWorkspace.evHu[97];
acadoWorkspace.A[1196] = acadoWorkspace.evHu[98];
acadoWorkspace.A[1197] = acadoWorkspace.evHu[99];
acadoWorkspace.A[1216] = acadoWorkspace.evHu[100];
acadoWorkspace.A[1217] = acadoWorkspace.evHu[101];
acadoWorkspace.A[1236] = acadoWorkspace.evHu[102];
acadoWorkspace.A[1237] = acadoWorkspace.evHu[103];
acadoWorkspace.A[1256] = acadoWorkspace.evHu[104];
acadoWorkspace.A[1257] = acadoWorkspace.evHu[105];
acadoWorkspace.A[1276] = acadoWorkspace.evHu[106];
acadoWorkspace.A[1277] = acadoWorkspace.evHu[107];
acadoWorkspace.A[1298] = acadoWorkspace.evHu[108];
acadoWorkspace.A[1299] = acadoWorkspace.evHu[109];
acadoWorkspace.A[1318] = acadoWorkspace.evHu[110];
acadoWorkspace.A[1319] = acadoWorkspace.evHu[111];
acadoWorkspace.A[1338] = acadoWorkspace.evHu[112];
acadoWorkspace.A[1339] = acadoWorkspace.evHu[113];
acadoWorkspace.A[1358] = acadoWorkspace.evHu[114];
acadoWorkspace.A[1359] = acadoWorkspace.evHu[115];
acadoWorkspace.A[1378] = acadoWorkspace.evHu[116];
acadoWorkspace.A[1379] = acadoWorkspace.evHu[117];
acadoWorkspace.A[1398] = acadoWorkspace.evHu[118];
acadoWorkspace.A[1399] = acadoWorkspace.evHu[119];
acadoWorkspace.lbA[10] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[0];
acadoWorkspace.lbA[11] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[1];
acadoWorkspace.lbA[12] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[2];
acadoWorkspace.lbA[13] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[3];
acadoWorkspace.lbA[14] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[4];
acadoWorkspace.lbA[15] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[5];
acadoWorkspace.lbA[16] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[6];
acadoWorkspace.lbA[17] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[7];
acadoWorkspace.lbA[18] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[8];
acadoWorkspace.lbA[19] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[9];
acadoWorkspace.lbA[20] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[10];
acadoWorkspace.lbA[21] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[11];
acadoWorkspace.lbA[22] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[12];
acadoWorkspace.lbA[23] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[13];
acadoWorkspace.lbA[24] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[14];
acadoWorkspace.lbA[25] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[15];
acadoWorkspace.lbA[26] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[16];
acadoWorkspace.lbA[27] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[17];
acadoWorkspace.lbA[28] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[18];
acadoWorkspace.lbA[29] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[19];
acadoWorkspace.lbA[30] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[20];
acadoWorkspace.lbA[31] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[21];
acadoWorkspace.lbA[32] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[22];
acadoWorkspace.lbA[33] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[23];
acadoWorkspace.lbA[34] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[24];
acadoWorkspace.lbA[35] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[25];
acadoWorkspace.lbA[36] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[26];
acadoWorkspace.lbA[37] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[27];
acadoWorkspace.lbA[38] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[28];
acadoWorkspace.lbA[39] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[29];
acadoWorkspace.lbA[40] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[30];
acadoWorkspace.lbA[41] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[31];
acadoWorkspace.lbA[42] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[32];
acadoWorkspace.lbA[43] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[33];
acadoWorkspace.lbA[44] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[34];
acadoWorkspace.lbA[45] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[35];
acadoWorkspace.lbA[46] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[36];
acadoWorkspace.lbA[47] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[37];
acadoWorkspace.lbA[48] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[38];
acadoWorkspace.lbA[49] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[39];
acadoWorkspace.lbA[50] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[40];
acadoWorkspace.lbA[51] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[41];
acadoWorkspace.lbA[52] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[42];
acadoWorkspace.lbA[53] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[43];
acadoWorkspace.lbA[54] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[44];
acadoWorkspace.lbA[55] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[45];
acadoWorkspace.lbA[56] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[46];
acadoWorkspace.lbA[57] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[47];
acadoWorkspace.lbA[58] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[48];
acadoWorkspace.lbA[59] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[49];
acadoWorkspace.lbA[60] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[50];
acadoWorkspace.lbA[61] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[51];
acadoWorkspace.lbA[62] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[52];
acadoWorkspace.lbA[63] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[53];
acadoWorkspace.lbA[64] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[54];
acadoWorkspace.lbA[65] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[55];
acadoWorkspace.lbA[66] = (real_t)-5.0000000000000000e+00 - acadoWorkspace.evH[56];
acadoWorkspace.lbA[67] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[57];
acadoWorkspace.lbA[68] = (real_t)-5.6999999999999995e-01 - acadoWorkspace.evH[58];
acadoWorkspace.lbA[69] = (real_t)-1.0000000000000000e+12 - acadoWorkspace.evH[59];

acadoWorkspace.ubA[10] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[0];
acadoWorkspace.ubA[11] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[1];
acadoWorkspace.ubA[12] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[2];
acadoWorkspace.ubA[13] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[3];
acadoWorkspace.ubA[14] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[4];
acadoWorkspace.ubA[15] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[5];
acadoWorkspace.ubA[16] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[6];
acadoWorkspace.ubA[17] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[7];
acadoWorkspace.ubA[18] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[8];
acadoWorkspace.ubA[19] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[9];
acadoWorkspace.ubA[20] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[10];
acadoWorkspace.ubA[21] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[11];
acadoWorkspace.ubA[22] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[12];
acadoWorkspace.ubA[23] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[13];
acadoWorkspace.ubA[24] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[14];
acadoWorkspace.ubA[25] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[15];
acadoWorkspace.ubA[26] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[16];
acadoWorkspace.ubA[27] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[17];
acadoWorkspace.ubA[28] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[18];
acadoWorkspace.ubA[29] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[19];
acadoWorkspace.ubA[30] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[20];
acadoWorkspace.ubA[31] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[21];
acadoWorkspace.ubA[32] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[22];
acadoWorkspace.ubA[33] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[23];
acadoWorkspace.ubA[34] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[24];
acadoWorkspace.ubA[35] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[25];
acadoWorkspace.ubA[36] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[26];
acadoWorkspace.ubA[37] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[27];
acadoWorkspace.ubA[38] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[28];
acadoWorkspace.ubA[39] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[29];
acadoWorkspace.ubA[40] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[30];
acadoWorkspace.ubA[41] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[31];
acadoWorkspace.ubA[42] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[32];
acadoWorkspace.ubA[43] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[33];
acadoWorkspace.ubA[44] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[34];
acadoWorkspace.ubA[45] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[35];
acadoWorkspace.ubA[46] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[36];
acadoWorkspace.ubA[47] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[37];
acadoWorkspace.ubA[48] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[38];
acadoWorkspace.ubA[49] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[39];
acadoWorkspace.ubA[50] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[40];
acadoWorkspace.ubA[51] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[41];
acadoWorkspace.ubA[52] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[42];
acadoWorkspace.ubA[53] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[43];
acadoWorkspace.ubA[54] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[44];
acadoWorkspace.ubA[55] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[45];
acadoWorkspace.ubA[56] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[46];
acadoWorkspace.ubA[57] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[47];
acadoWorkspace.ubA[58] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[48];
acadoWorkspace.ubA[59] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[49];
acadoWorkspace.ubA[60] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[50];
acadoWorkspace.ubA[61] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[51];
acadoWorkspace.ubA[62] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[52];
acadoWorkspace.ubA[63] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[53];
acadoWorkspace.ubA[64] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[54];
acadoWorkspace.ubA[65] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[55];
acadoWorkspace.ubA[66] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[56];
acadoWorkspace.ubA[67] = (real_t)5.0000000000000000e+00 - acadoWorkspace.evH[57];
acadoWorkspace.ubA[68] = (real_t)1.0000000000000000e+12 - acadoWorkspace.evH[58];
acadoWorkspace.ubA[69] = (real_t)5.6999999999999995e-01 - acadoWorkspace.evH[59];

}

void acado_condenseFdb(  )
{
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.Dy[105] -= acadoVariables.y[105];
acadoWorkspace.Dy[106] -= acadoVariables.y[106];
acadoWorkspace.Dy[107] -= acadoVariables.y[107];
acadoWorkspace.Dy[108] -= acadoVariables.y[108];
acadoWorkspace.Dy[109] -= acadoVariables.y[109];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 22 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.g[ 2 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 44 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 66 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 88 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 110 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.g[ 10 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 132 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 154 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.g[ 14 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 176 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 198 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.g[ 18 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 55 ]), &(acadoWorkspace.Dy[ 11 ]), &(acadoWorkspace.QDy[ 5 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 110 ]), &(acadoWorkspace.Dy[ 22 ]), &(acadoWorkspace.QDy[ 10 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 165 ]), &(acadoWorkspace.Dy[ 33 ]), &(acadoWorkspace.QDy[ 15 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 220 ]), &(acadoWorkspace.Dy[ 44 ]), &(acadoWorkspace.QDy[ 20 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 275 ]), &(acadoWorkspace.Dy[ 55 ]), &(acadoWorkspace.QDy[ 25 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 330 ]), &(acadoWorkspace.Dy[ 66 ]), &(acadoWorkspace.QDy[ 30 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 385 ]), &(acadoWorkspace.Dy[ 77 ]), &(acadoWorkspace.QDy[ 35 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 440 ]), &(acadoWorkspace.Dy[ 88 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 495 ]), &(acadoWorkspace.Dy[ 99 ]), &(acadoWorkspace.QDy[ 45 ]) );

acadoWorkspace.QDy[50] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[51] = + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[52] = + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[53] = + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[6];
acadoWorkspace.QDy[54] = + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[6];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 5 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 50 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[50] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[51] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[52] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[53] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[54] + acadoWorkspace.QDy[50];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[50] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[51] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[52] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[53] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[54] + acadoWorkspace.QDy[51];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[50] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[51] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[52] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[53] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[54] + acadoWorkspace.QDy[52];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[50] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[51] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[52] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[53] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[54] + acadoWorkspace.QDy[53];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[50] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[51] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[52] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[53] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[54] + acadoWorkspace.QDy[54];
acado_macBTw1( &(acadoWorkspace.evGu[ 90 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 225 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 225 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 80 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 200 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 200 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 70 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 14 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 175 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 35 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 175 ]), &(acadoWorkspace.sbar[ 35 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 60 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 150 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 30 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 150 ]), &(acadoWorkspace.sbar[ 30 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 50 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 10 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 125 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 25 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 125 ]), &(acadoWorkspace.sbar[ 25 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 40 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 100 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 20 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 100 ]), &(acadoWorkspace.sbar[ 20 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 30 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 75 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 15 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 75 ]), &(acadoWorkspace.sbar[ 15 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 20 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 50 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 10 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 50 ]), &(acadoWorkspace.sbar[ 10 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 10 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 2 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 25 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 5 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 25 ]), &(acadoWorkspace.sbar[ 5 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[9] + acadoVariables.x[9];
acadoWorkspace.lbA[0] = - tmp;
acadoWorkspace.ubA[0] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[14] + acadoVariables.x[14];
acadoWorkspace.lbA[1] = - tmp;
acadoWorkspace.ubA[1] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[19] + acadoVariables.x[19];
acadoWorkspace.lbA[2] = - tmp;
acadoWorkspace.ubA[2] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[3] = - tmp;
acadoWorkspace.ubA[3] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[29] + acadoVariables.x[29];
acadoWorkspace.lbA[4] = - tmp;
acadoWorkspace.ubA[4] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[5] = - tmp;
acadoWorkspace.ubA[5] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[39] + acadoVariables.x[39];
acadoWorkspace.lbA[6] = - tmp;
acadoWorkspace.ubA[6] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[7] = - tmp;
acadoWorkspace.ubA[7] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[8] = - tmp;
acadoWorkspace.ubA[8] = (real_t)1.0000000000000000e+12 - tmp;
tmp = acadoWorkspace.sbar[54] + acadoVariables.x[54];
acadoWorkspace.lbA[9] = - tmp;
acadoWorkspace.ubA[9] = (real_t)1.0000000000000000e+12 - tmp;

acado_macHxd( acadoWorkspace.evHx, acadoWorkspace.sbar, &(acadoWorkspace.lbA[ 10 ]), &(acadoWorkspace.ubA[ 10 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 30 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.lbA[ 16 ]), &(acadoWorkspace.ubA[ 16 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 60 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.lbA[ 22 ]), &(acadoWorkspace.ubA[ 22 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 90 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.lbA[ 28 ]), &(acadoWorkspace.ubA[ 28 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 120 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.lbA[ 34 ]), &(acadoWorkspace.ubA[ 34 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 150 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.lbA[ 40 ]), &(acadoWorkspace.ubA[ 40 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 180 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.lbA[ 46 ]), &(acadoWorkspace.ubA[ 46 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 210 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.lbA[ 52 ]), &(acadoWorkspace.ubA[ 52 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 240 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.lbA[ 58 ]), &(acadoWorkspace.ubA[ 58 ]) );
acado_macHxd( &(acadoWorkspace.evHx[ 270 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.lbA[ 64 ]), &(acadoWorkspace.ubA[ 64 ]) );

}

void acado_expand(  )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.d[0];
acadoWorkspace.sbar[6] = acadoWorkspace.d[1];
acadoWorkspace.sbar[7] = acadoWorkspace.d[2];
acadoWorkspace.sbar[8] = acadoWorkspace.d[3];
acadoWorkspace.sbar[9] = acadoWorkspace.d[4];
acadoWorkspace.sbar[10] = acadoWorkspace.d[5];
acadoWorkspace.sbar[11] = acadoWorkspace.d[6];
acadoWorkspace.sbar[12] = acadoWorkspace.d[7];
acadoWorkspace.sbar[13] = acadoWorkspace.d[8];
acadoWorkspace.sbar[14] = acadoWorkspace.d[9];
acadoWorkspace.sbar[15] = acadoWorkspace.d[10];
acadoWorkspace.sbar[16] = acadoWorkspace.d[11];
acadoWorkspace.sbar[17] = acadoWorkspace.d[12];
acadoWorkspace.sbar[18] = acadoWorkspace.d[13];
acadoWorkspace.sbar[19] = acadoWorkspace.d[14];
acadoWorkspace.sbar[20] = acadoWorkspace.d[15];
acadoWorkspace.sbar[21] = acadoWorkspace.d[16];
acadoWorkspace.sbar[22] = acadoWorkspace.d[17];
acadoWorkspace.sbar[23] = acadoWorkspace.d[18];
acadoWorkspace.sbar[24] = acadoWorkspace.d[19];
acadoWorkspace.sbar[25] = acadoWorkspace.d[20];
acadoWorkspace.sbar[26] = acadoWorkspace.d[21];
acadoWorkspace.sbar[27] = acadoWorkspace.d[22];
acadoWorkspace.sbar[28] = acadoWorkspace.d[23];
acadoWorkspace.sbar[29] = acadoWorkspace.d[24];
acadoWorkspace.sbar[30] = acadoWorkspace.d[25];
acadoWorkspace.sbar[31] = acadoWorkspace.d[26];
acadoWorkspace.sbar[32] = acadoWorkspace.d[27];
acadoWorkspace.sbar[33] = acadoWorkspace.d[28];
acadoWorkspace.sbar[34] = acadoWorkspace.d[29];
acadoWorkspace.sbar[35] = acadoWorkspace.d[30];
acadoWorkspace.sbar[36] = acadoWorkspace.d[31];
acadoWorkspace.sbar[37] = acadoWorkspace.d[32];
acadoWorkspace.sbar[38] = acadoWorkspace.d[33];
acadoWorkspace.sbar[39] = acadoWorkspace.d[34];
acadoWorkspace.sbar[40] = acadoWorkspace.d[35];
acadoWorkspace.sbar[41] = acadoWorkspace.d[36];
acadoWorkspace.sbar[42] = acadoWorkspace.d[37];
acadoWorkspace.sbar[43] = acadoWorkspace.d[38];
acadoWorkspace.sbar[44] = acadoWorkspace.d[39];
acadoWorkspace.sbar[45] = acadoWorkspace.d[40];
acadoWorkspace.sbar[46] = acadoWorkspace.d[41];
acadoWorkspace.sbar[47] = acadoWorkspace.d[42];
acadoWorkspace.sbar[48] = acadoWorkspace.d[43];
acadoWorkspace.sbar[49] = acadoWorkspace.d[44];
acadoWorkspace.sbar[50] = acadoWorkspace.d[45];
acadoWorkspace.sbar[51] = acadoWorkspace.d[46];
acadoWorkspace.sbar[52] = acadoWorkspace.d[47];
acadoWorkspace.sbar[53] = acadoWorkspace.d[48];
acadoWorkspace.sbar[54] = acadoWorkspace.d[49];
acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 5 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 25 ]), &(acadoWorkspace.evGu[ 10 ]), &(acadoWorkspace.x[ 2 ]), &(acadoWorkspace.sbar[ 5 ]), &(acadoWorkspace.sbar[ 10 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 50 ]), &(acadoWorkspace.evGu[ 20 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 10 ]), &(acadoWorkspace.sbar[ 15 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 75 ]), &(acadoWorkspace.evGu[ 30 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 15 ]), &(acadoWorkspace.sbar[ 20 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 100 ]), &(acadoWorkspace.evGu[ 40 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 20 ]), &(acadoWorkspace.sbar[ 25 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 125 ]), &(acadoWorkspace.evGu[ 50 ]), &(acadoWorkspace.x[ 10 ]), &(acadoWorkspace.sbar[ 25 ]), &(acadoWorkspace.sbar[ 30 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 150 ]), &(acadoWorkspace.evGu[ 60 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 30 ]), &(acadoWorkspace.sbar[ 35 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 175 ]), &(acadoWorkspace.evGu[ 70 ]), &(acadoWorkspace.x[ 14 ]), &(acadoWorkspace.sbar[ 35 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 200 ]), &(acadoWorkspace.evGu[ 80 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 225 ]), &(acadoWorkspace.evGu[ 90 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 50 ]) );
acadoVariables.x[0] += acadoWorkspace.sbar[0];
acadoVariables.x[1] += acadoWorkspace.sbar[1];
acadoVariables.x[2] += acadoWorkspace.sbar[2];
acadoVariables.x[3] += acadoWorkspace.sbar[3];
acadoVariables.x[4] += acadoWorkspace.sbar[4];
acadoVariables.x[5] += acadoWorkspace.sbar[5];
acadoVariables.x[6] += acadoWorkspace.sbar[6];
acadoVariables.x[7] += acadoWorkspace.sbar[7];
acadoVariables.x[8] += acadoWorkspace.sbar[8];
acadoVariables.x[9] += acadoWorkspace.sbar[9];
acadoVariables.x[10] += acadoWorkspace.sbar[10];
acadoVariables.x[11] += acadoWorkspace.sbar[11];
acadoVariables.x[12] += acadoWorkspace.sbar[12];
acadoVariables.x[13] += acadoWorkspace.sbar[13];
acadoVariables.x[14] += acadoWorkspace.sbar[14];
acadoVariables.x[15] += acadoWorkspace.sbar[15];
acadoVariables.x[16] += acadoWorkspace.sbar[16];
acadoVariables.x[17] += acadoWorkspace.sbar[17];
acadoVariables.x[18] += acadoWorkspace.sbar[18];
acadoVariables.x[19] += acadoWorkspace.sbar[19];
acadoVariables.x[20] += acadoWorkspace.sbar[20];
acadoVariables.x[21] += acadoWorkspace.sbar[21];
acadoVariables.x[22] += acadoWorkspace.sbar[22];
acadoVariables.x[23] += acadoWorkspace.sbar[23];
acadoVariables.x[24] += acadoWorkspace.sbar[24];
acadoVariables.x[25] += acadoWorkspace.sbar[25];
acadoVariables.x[26] += acadoWorkspace.sbar[26];
acadoVariables.x[27] += acadoWorkspace.sbar[27];
acadoVariables.x[28] += acadoWorkspace.sbar[28];
acadoVariables.x[29] += acadoWorkspace.sbar[29];
acadoVariables.x[30] += acadoWorkspace.sbar[30];
acadoVariables.x[31] += acadoWorkspace.sbar[31];
acadoVariables.x[32] += acadoWorkspace.sbar[32];
acadoVariables.x[33] += acadoWorkspace.sbar[33];
acadoVariables.x[34] += acadoWorkspace.sbar[34];
acadoVariables.x[35] += acadoWorkspace.sbar[35];
acadoVariables.x[36] += acadoWorkspace.sbar[36];
acadoVariables.x[37] += acadoWorkspace.sbar[37];
acadoVariables.x[38] += acadoWorkspace.sbar[38];
acadoVariables.x[39] += acadoWorkspace.sbar[39];
acadoVariables.x[40] += acadoWorkspace.sbar[40];
acadoVariables.x[41] += acadoWorkspace.sbar[41];
acadoVariables.x[42] += acadoWorkspace.sbar[42];
acadoVariables.x[43] += acadoWorkspace.sbar[43];
acadoVariables.x[44] += acadoWorkspace.sbar[44];
acadoVariables.x[45] += acadoWorkspace.sbar[45];
acadoVariables.x[46] += acadoWorkspace.sbar[46];
acadoVariables.x[47] += acadoWorkspace.sbar[47];
acadoVariables.x[48] += acadoWorkspace.sbar[48];
acadoVariables.x[49] += acadoWorkspace.sbar[49];
acadoVariables.x[50] += acadoWorkspace.sbar[50];
acadoVariables.x[51] += acadoWorkspace.sbar[51];
acadoVariables.x[52] += acadoWorkspace.sbar[52];
acadoVariables.x[53] += acadoWorkspace.sbar[53];
acadoVariables.x[54] += acadoWorkspace.sbar[54];
}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 5];
acadoWorkspace.state[1] = acadoVariables.x[index * 5 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 5 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 5 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 5 + 4];
acadoWorkspace.state[40] = acadoVariables.u[index * 2];
acadoWorkspace.state[41] = acadoVariables.u[index * 2 + 1];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 5 + 5] = acadoWorkspace.state[0];
acadoVariables.x[index * 5 + 6] = acadoWorkspace.state[1];
acadoVariables.x[index * 5 + 7] = acadoWorkspace.state[2];
acadoVariables.x[index * 5 + 8] = acadoWorkspace.state[3];
acadoVariables.x[index * 5 + 9] = acadoWorkspace.state[4];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 5] = acadoVariables.x[index * 5 + 5];
acadoVariables.x[index * 5 + 1] = acadoVariables.x[index * 5 + 6];
acadoVariables.x[index * 5 + 2] = acadoVariables.x[index * 5 + 7];
acadoVariables.x[index * 5 + 3] = acadoVariables.x[index * 5 + 8];
acadoVariables.x[index * 5 + 4] = acadoVariables.x[index * 5 + 9];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[50] = xEnd[0];
acadoVariables.x[51] = xEnd[1];
acadoVariables.x[52] = xEnd[2];
acadoVariables.x[53] = xEnd[3];
acadoVariables.x[54] = xEnd[4];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[50];
acadoWorkspace.state[1] = acadoVariables.x[51];
acadoWorkspace.state[2] = acadoVariables.x[52];
acadoWorkspace.state[3] = acadoVariables.x[53];
acadoWorkspace.state[4] = acadoVariables.x[54];
if (uEnd != 0)
{
acadoWorkspace.state[40] = uEnd[0];
acadoWorkspace.state[41] = uEnd[1];
}
else
{
acadoWorkspace.state[40] = acadoVariables.u[18];
acadoWorkspace.state[41] = acadoVariables.u[19];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[50] = acadoWorkspace.state[0];
acadoVariables.x[51] = acadoWorkspace.state[1];
acadoVariables.x[52] = acadoWorkspace.state[2];
acadoVariables.x[53] = acadoWorkspace.state[3];
acadoVariables.x[54] = acadoWorkspace.state[4];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 2] = acadoVariables.u[index * 2 + 2];
acadoVariables.u[index * 2 + 1] = acadoVariables.u[index * 2 + 3];
}

if (uEnd != 0)
{
acadoVariables.u[18] = uEnd[0];
acadoVariables.u[19] = uEnd[1];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19];
kkt = fabs( kkt );
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 70; ++index)
{
prd = acadoWorkspace.y[index + 20];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 11 */
real_t tmpDy[ 11 ];

/** Row vector of size: 7 */
real_t tmpDyN[ 7 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 5];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 5 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 5 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 5 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 5 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.u[lRun1 * 2];
acadoWorkspace.objValueIn[6] = acadoVariables.u[lRun1 * 2 + 1];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 11] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 11];
acadoWorkspace.Dy[lRun1 * 11 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 11 + 1];
acadoWorkspace.Dy[lRun1 * 11 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 11 + 2];
acadoWorkspace.Dy[lRun1 * 11 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 11 + 3];
acadoWorkspace.Dy[lRun1 * 11 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 11 + 4];
acadoWorkspace.Dy[lRun1 * 11 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 11 + 5];
acadoWorkspace.Dy[lRun1 * 11 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 11 + 6];
acadoWorkspace.Dy[lRun1 * 11 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 11 + 7];
acadoWorkspace.Dy[lRun1 * 11 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 11 + 8];
acadoWorkspace.Dy[lRun1 * 11 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 11 + 9];
acadoWorkspace.Dy[lRun1 * 11 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 11 + 10];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[50];
acadoWorkspace.objValueIn[1] = acadoVariables.x[51];
acadoWorkspace.objValueIn[2] = acadoVariables.x[52];
acadoWorkspace.objValueIn[3] = acadoVariables.x[53];
acadoWorkspace.objValueIn[4] = acadoVariables.x[54];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 11];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 11 + 1];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 11 + 2];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 11 + 3];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 11 + 4];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 11 + 5];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 11 + 6];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 11 + 7];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 11 + 8];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 11 + 9];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 11 + 10];
objVal += + acadoWorkspace.Dy[lRun1 * 11]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 11 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 11 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 11 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 11 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 11 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 11 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 11 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 11 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 11 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 11 + 10]*tmpDy[10];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*(real_t)1.0000000000000000e+01;
tmpDyN[1] = + acadoWorkspace.DyN[1]*(real_t)1.0000000000000000e+01;
tmpDyN[2] = + acadoWorkspace.DyN[2]*(real_t)1.0000000000000000e+01;
tmpDyN[3] = + acadoWorkspace.DyN[3]*(real_t)1.0000000000000000e+01;
tmpDyN[4] = + acadoWorkspace.DyN[4]*(real_t)1.0000000000000000e+01;
tmpDyN[5] = + acadoWorkspace.DyN[5]*(real_t)1.0000000000000000e+01;
tmpDyN[6] = + acadoWorkspace.DyN[6]*(real_t)1.0000000000000000e+01;
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6];

objVal *= 0.5;
return objVal;
}

