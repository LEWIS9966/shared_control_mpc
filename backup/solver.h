/* Produced by CVXGEN, 2022-09-26 12:22:00 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif

#ifdef __cplusplus
 extern "C" {
#endif


/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double x_0[8];
  double x_ss_0[8];
  double Q_x[64];
  double u_ss_0[3];
  double R_u[9];
  double u_prev[3];
  double R_delta[9];
  double x_ss_1[8];
  double u_ss_1[3];
  double x_ss_2[8];
  double u_ss_2[3];
  double x_ss_3[8];
  double u_ss_3[3];
  double x_ss_4[8];
  double u_ss_4[3];
  double x_ss_5[8];
  double u_ss_5[3];
  double x_ss_6[8];
  double u_ss_6[3];
  double x_ss_7[8];
  double u_ss_7[3];
  double x_ss_8[8];
  double u_ss_8[3];
  double x_ss_9[8];
  double u_ss_9[3];
  double x_ss_10[8];
  double u_ss_10[3];
  double x_ss_11[8];
  double u_ss_11[3];
  double x_ss_12[8];
  double u_ss_12[3];
  double x_ss_13[8];
  double u_ss_13[3];
  double x_ss_14[8];
  double u_ss_14[3];
  double x_ss_15[8];
  double u_ss_15[3];
  double x_ss_16[8];
  double u_ss_16[3];
  double x_ss_17[8];
  double u_ss_17[3];
  double x_ss_18[8];
  double u_ss_18[3];
  double x_ss_19[8];
  double u_ss_19[3];
  double x_ss_20[8];
  double u_ss_20[3];
  double x_ss_21[8];
  double P[64];
  double A[64];
  double B[24];
  double u_min[3];
  double u_max[3];
  double *x[1];
  double *x_ss[22];
  double *u_ss[21];
} Params;
typedef struct Vars_t {
  double *u_0; /* 3 rows. */
  double *x_1; /* 8 rows. */
  double *u_1; /* 3 rows. */
  double *t_01; /* 3 rows. */
  double *x_2; /* 8 rows. */
  double *u_2; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *x_3; /* 8 rows. */
  double *u_3; /* 3 rows. */
  double *t_03; /* 3 rows. */
  double *x_4; /* 8 rows. */
  double *u_4; /* 3 rows. */
  double *t_04; /* 3 rows. */
  double *x_5; /* 8 rows. */
  double *u_5; /* 3 rows. */
  double *t_05; /* 3 rows. */
  double *x_6; /* 8 rows. */
  double *u_6; /* 3 rows. */
  double *t_06; /* 3 rows. */
  double *x_7; /* 8 rows. */
  double *u_7; /* 3 rows. */
  double *t_07; /* 3 rows. */
  double *x_8; /* 8 rows. */
  double *u_8; /* 3 rows. */
  double *t_08; /* 3 rows. */
  double *x_9; /* 8 rows. */
  double *u_9; /* 3 rows. */
  double *t_09; /* 3 rows. */
  double *x_10; /* 8 rows. */
  double *u_10; /* 3 rows. */
  double *t_10; /* 3 rows. */
  double *x_11; /* 8 rows. */
  double *u_11; /* 3 rows. */
  double *t_11; /* 3 rows. */
  double *x_12; /* 8 rows. */
  double *u_12; /* 3 rows. */
  double *t_12; /* 3 rows. */
  double *x_13; /* 8 rows. */
  double *u_13; /* 3 rows. */
  double *t_13; /* 3 rows. */
  double *x_14; /* 8 rows. */
  double *u_14; /* 3 rows. */
  double *t_14; /* 3 rows. */
  double *x_15; /* 8 rows. */
  double *u_15; /* 3 rows. */
  double *t_15; /* 3 rows. */
  double *x_16; /* 8 rows. */
  double *u_16; /* 3 rows. */
  double *t_16; /* 3 rows. */
  double *x_17; /* 8 rows. */
  double *u_17; /* 3 rows. */
  double *t_17; /* 3 rows. */
  double *x_18; /* 8 rows. */
  double *u_18; /* 3 rows. */
  double *t_18; /* 3 rows. */
  double *x_19; /* 8 rows. */
  double *u_19; /* 3 rows. */
  double *t_19; /* 3 rows. */
  double *x_20; /* 8 rows. */
  double *u_20; /* 3 rows. */
  double *t_20; /* 3 rows. */
  double *x_21; /* 8 rows. */
  double *u[21];
  double *x[22];
} Vars;
typedef struct Workspace_t {
  double h[126];
  double s_inv[126];
  double s_inv_z[126];
  double b[228];
  double q[291];
  double rhs[771];
  double x[771];
  double *s;
  double *z;
  double *y;
  double lhs_aff[771];
  double lhs_cc[771];
  double buffer[771];
  double buffer2[771];
  double KKT[3638];
  double L[6355];
  double d[771];
  double v[771];
  double d_inv[771];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_345851330560[1];
  double quad_503357964288[1];
  double quad_689417555968[1];
  double quad_590025695232[1];
  double quad_24887435264[1];
  double quad_373406756864[1];
  double quad_282534416384[1];
  double quad_569492807680[1];
  double quad_658085392384[1];
  double quad_268442972160[1];
  double quad_212760903680[1];
  double quad_604237524992[1];
  double quad_899615596544[1];
  double quad_9807298560[1];
  double quad_730290806784[1];
  double quad_691068739584[1];
  double quad_512929845248[1];
  double quad_46428270592[1];
  double quad_907958816768[1];
  double quad_364190564352[1];
  double quad_603410014208[1];
  double quad_253294055424[1];
  double quad_406480764928[1];
  double quad_450189660160[1];
  double quad_622448181248[1];
  double quad_135073144832[1];
  double quad_512757641216[1];
  double quad_265658822656[1];
  double quad_225439469568[1];
  double quad_611752235008[1];
  double quad_945124098048[1];
  double quad_625116979200[1];
  double quad_566313590784[1];
  double quad_323441446912[1];
  double quad_589385719808[1];
  double quad_920610275328[1];
  double quad_700597231616[1];
  double quad_379984363520[1];
  double quad_438523551744[1];
  double quad_43660177408[1];
  double quad_837056782336[1];
  double quad_396244647936[1];
  double quad_326908334080[1];
  double quad_28181692416[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#ifdef __cplusplus
 }
#endif

#endif
