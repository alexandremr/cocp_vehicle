/* Produced by CVXGEN, 2020-09-17 15:09:18 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.lam_3[0] = 1.101595805149151;
  params.lam_4[0] = 1.4162956452362097;
  params.P_sqrt[0] = -0.8363810443482227;
  params.P_sqrt[1] = 0.04331042079065206;
  params.P_sqrt[2] = 1.5717878173906188;
  params.P_sqrt[3] = 1.5851723557337523;
  params.P_sqrt[4] = -1.497658758144655;
  params.P_sqrt[5] = -1.171028487447253;
  params.P_sqrt[6] = -1.7941311867966805;
  params.P_sqrt[7] = -0.23676062539745413;
  params.P_sqrt[8] = -1.8804951564857322;
  params.P_sqrt[9] = -0.17266710242115568;
  params.P_sqrt[10] = 0.596576190459043;
  params.P_sqrt[11] = -0.8860508694080989;
  params.P_sqrt[12] = 0.7050196079205251;
  params.P_sqrt[13] = 0.3634512696654033;
  params.P_sqrt[14] = -1.9040724704913385;
  params.P_sqrt[15] = 0.23541635196352795;
  params.q[0] = -0.9629902123701384;
  params.q[1] = -0.3395952119597214;
  params.q[2] = -0.865899672914725;
  params.q[3] = 0.7725516732519853;
  params.fx[0] = -0.23818512931704205;
  params.fx[1] = -1.372529046100147;
  params.fx[2] = 0.17859607212737894;
  params.fx[3] = 1.1212590580454682;
  params.B[0] = -0.774545870495281;
  params.B[1] = -1.1121684642712744;
  params.B[2] = -0.44811496977740495;
  params.B[3] = 1.7455345994417217;
  params.B[4] = 1.9039816898917352;
  params.B[5] = 0.6895347036512547;
  params.B[6] = 1.6113364341535923;
  params.B[7] = 1.383003485172717;
  params.a_max[0] = -0.48802383468444344;
  params.L[0] = -1.631131964513103;
  params.k[0] = 0.6136436100941447;
  params.tan_d_max[0] = 0.2313630495538037;
  params.z_prev[0] = -0.5537409477496875;
  params.k_prev[0] = -1.0997819806406723;
  params.dtan_max[0] = -0.3739203344950055;
}
