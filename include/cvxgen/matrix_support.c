/* Produced by CVXGEN, 2020-09-17 15:09:17 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1)-rhs[4]*(1);
  lhs[1] = -rhs[1]*(-1)-rhs[5]*(1);
  lhs[2] = -rhs[4]*(-params.B[0])-rhs[5]*(-params.B[4])-rhs[6]*(1);
  lhs[3] = -rhs[4]*(-params.B[1])-rhs[5]*(-params.B[5])-rhs[7]*(1);
  lhs[4] = -rhs[4]*(-params.B[2])-rhs[5]*(-params.B[6])-rhs[8]*(1);
  lhs[5] = -rhs[4]*(-params.B[3])-rhs[5]*(-params.B[7])-rhs[9]*(1);
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-1);
  lhs[1] = -rhs[1]*(-1);
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = -rhs[0]*(1)-rhs[2]*(-params.B[0])-rhs[3]*(-params.B[1])-rhs[4]*(-params.B[2])-rhs[5]*(-params.B[3]);
  lhs[5] = -rhs[1]*(1)-rhs[2]*(-params.B[4])-rhs[3]*(-params.B[5])-rhs[4]*(-params.B[6])-rhs[5]*(-params.B[7]);
  lhs[6] = -rhs[2]*(1);
  lhs[7] = -rhs[3]*(1);
  lhs[8] = -rhs[4]*(1);
  lhs[9] = -rhs[5]*(1);
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[4]*(-1);
  lhs[1] = -rhs[4]*(1);
  lhs[2] = -rhs[2]*(1);
  lhs[3] = -rhs[2]*(-1)-rhs[5]*(1);
  lhs[4] = -rhs[2]*(-1)-rhs[5]*(-1);
  lhs[5] = -rhs[3]*(1);
  lhs[6] = -rhs[3]*(-1)-rhs[5]*(1);
  lhs[7] = -rhs[3]*(-1)-rhs[5]*(-1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = -rhs[2]*(1)-rhs[3]*(-1)-rhs[4]*(-1);
  lhs[3] = -rhs[5]*(1)-rhs[6]*(-1)-rhs[7]*(-1);
  lhs[4] = -rhs[0]*(-1)-rhs[1]*(1);
  lhs[5] = -rhs[3]*(1)-rhs[4]*(-1)-rhs[6]*(1)-rhs[7]*(-1);
  lhs[6] = 0;
  lhs[7] = 0;
  lhs[8] = 0;
  lhs[9] = 0;
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.lam_3[0]);
  lhs[1] = rhs[1]*(2*params.lam_4[0]);
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
  lhs[6] = rhs[6]*(2*work.quad_68573827072[0])+rhs[7]*(2*work.quad_68573827072[4])+rhs[8]*(2*work.quad_68573827072[8])+rhs[9]*(2*work.quad_68573827072[12]);
  lhs[7] = rhs[6]*(2*work.quad_68573827072[1])+rhs[7]*(2*work.quad_68573827072[5])+rhs[8]*(2*work.quad_68573827072[9])+rhs[9]*(2*work.quad_68573827072[13]);
  lhs[8] = rhs[6]*(2*work.quad_68573827072[2])+rhs[7]*(2*work.quad_68573827072[6])+rhs[8]*(2*work.quad_68573827072[10])+rhs[9]*(2*work.quad_68573827072[14]);
  lhs[9] = rhs[6]*(2*work.quad_68573827072[3])+rhs[7]*(2*work.quad_68573827072[7])+rhs[8]*(2*work.quad_68573827072[11])+rhs[9]*(2*work.quad_68573827072[15]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
  work.q[6] = params.q[0];
  work.q[7] = params.q[1];
  work.q[8] = params.q[2];
  work.q[9] = params.q[3];
}
void fillh(void) {
  work.h[0] = params.a_max[0];
  work.h[1] = params.a_max[0];
  work.h[2] = params.tan_d_max[0];
  work.h[3] = -params.L[0]*params.k[0];
  work.h[4] = params.L[0]*params.k[0];
  work.h[5] = params.dtan_max[0];
  work.h[6] = -(params.L[0]*params.k[0]-params.z_prev[0]-params.L[0]*params.k_prev[0]);
  work.h[7] = params.L[0]*params.k[0]-params.z_prev[0]-params.L[0]*params.k_prev[0];
}
void fillb(void) {
  work.b[0] = 0;
  work.b[1] = 0;
  work.b[2] = params.fx[0];
  work.b[3] = params.fx[1];
  work.b[4] = params.fx[2];
  work.b[5] = params.fx[3];
}
void pre_ops(void) {
  work.quad_68573827072[0] = params.P_sqrt[0]*params.P_sqrt[0]+params.P_sqrt[1]*params.P_sqrt[1]+params.P_sqrt[2]*params.P_sqrt[2]+params.P_sqrt[3]*params.P_sqrt[3];
  work.quad_68573827072[4] = params.P_sqrt[0]*params.P_sqrt[4]+params.P_sqrt[1]*params.P_sqrt[5]+params.P_sqrt[2]*params.P_sqrt[6]+params.P_sqrt[3]*params.P_sqrt[7];
  work.quad_68573827072[8] = params.P_sqrt[0]*params.P_sqrt[8]+params.P_sqrt[1]*params.P_sqrt[9]+params.P_sqrt[2]*params.P_sqrt[10]+params.P_sqrt[3]*params.P_sqrt[11];
  work.quad_68573827072[12] = params.P_sqrt[0]*params.P_sqrt[12]+params.P_sqrt[1]*params.P_sqrt[13]+params.P_sqrt[2]*params.P_sqrt[14]+params.P_sqrt[3]*params.P_sqrt[15];
  work.quad_68573827072[1] = params.P_sqrt[4]*params.P_sqrt[0]+params.P_sqrt[5]*params.P_sqrt[1]+params.P_sqrt[6]*params.P_sqrt[2]+params.P_sqrt[7]*params.P_sqrt[3];
  work.quad_68573827072[5] = params.P_sqrt[4]*params.P_sqrt[4]+params.P_sqrt[5]*params.P_sqrt[5]+params.P_sqrt[6]*params.P_sqrt[6]+params.P_sqrt[7]*params.P_sqrt[7];
  work.quad_68573827072[9] = params.P_sqrt[4]*params.P_sqrt[8]+params.P_sqrt[5]*params.P_sqrt[9]+params.P_sqrt[6]*params.P_sqrt[10]+params.P_sqrt[7]*params.P_sqrt[11];
  work.quad_68573827072[13] = params.P_sqrt[4]*params.P_sqrt[12]+params.P_sqrt[5]*params.P_sqrt[13]+params.P_sqrt[6]*params.P_sqrt[14]+params.P_sqrt[7]*params.P_sqrt[15];
  work.quad_68573827072[2] = params.P_sqrt[8]*params.P_sqrt[0]+params.P_sqrt[9]*params.P_sqrt[1]+params.P_sqrt[10]*params.P_sqrt[2]+params.P_sqrt[11]*params.P_sqrt[3];
  work.quad_68573827072[6] = params.P_sqrt[8]*params.P_sqrt[4]+params.P_sqrt[9]*params.P_sqrt[5]+params.P_sqrt[10]*params.P_sqrt[6]+params.P_sqrt[11]*params.P_sqrt[7];
  work.quad_68573827072[10] = params.P_sqrt[8]*params.P_sqrt[8]+params.P_sqrt[9]*params.P_sqrt[9]+params.P_sqrt[10]*params.P_sqrt[10]+params.P_sqrt[11]*params.P_sqrt[11];
  work.quad_68573827072[14] = params.P_sqrt[8]*params.P_sqrt[12]+params.P_sqrt[9]*params.P_sqrt[13]+params.P_sqrt[10]*params.P_sqrt[14]+params.P_sqrt[11]*params.P_sqrt[15];
  work.quad_68573827072[3] = params.P_sqrt[12]*params.P_sqrt[0]+params.P_sqrt[13]*params.P_sqrt[1]+params.P_sqrt[14]*params.P_sqrt[2]+params.P_sqrt[15]*params.P_sqrt[3];
  work.quad_68573827072[7] = params.P_sqrt[12]*params.P_sqrt[4]+params.P_sqrt[13]*params.P_sqrt[5]+params.P_sqrt[14]*params.P_sqrt[6]+params.P_sqrt[15]*params.P_sqrt[7];
  work.quad_68573827072[11] = params.P_sqrt[12]*params.P_sqrt[8]+params.P_sqrt[13]*params.P_sqrt[9]+params.P_sqrt[14]*params.P_sqrt[10]+params.P_sqrt[15]*params.P_sqrt[11];
  work.quad_68573827072[15] = params.P_sqrt[12]*params.P_sqrt[12]+params.P_sqrt[13]*params.P_sqrt[13]+params.P_sqrt[14]*params.P_sqrt[14]+params.P_sqrt[15]*params.P_sqrt[15];
}
