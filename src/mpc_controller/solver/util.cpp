#include <mrs_mpc_solvers/controller/solver/solver.h>

#include <time.h>
#include <stdlib.h>
#include <math.h>

namespace mrs_mpc_solvers
{

namespace mpc_controller
{

void QPSolver::tic_controller(void) {
  tic_timestart = clock();
}

float QPSolver::toc_controller(void) {
  clock_t tic_timestop;
  tic_timestop = clock();
  printf("time: %8.2f.\n", (float)(tic_timestop - tic_timestart) / CLOCKS_PER_SEC);
  return (float)(tic_timestop - tic_timestart) / CLOCKS_PER_SEC;
}

float QPSolver::tocq_controller(void) {
  clock_t tic_timestop;
  tic_timestop = clock();
  return (float)(tic_timestop - tic_timestart) / CLOCKS_PER_SEC;
}

void QPSolver::printmatrixd_controller(char *name, double *A, int m, int n, int sparse) {
  int i, j;
  printf("%s = [...\n", name);
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++)
      if ((sparse == 1) && (A[i + j * m] == 0))
        printf("         0");
      else
        printf("  % 9.4f", A[i + j * m]);
    printf(",\n");
  }
  printf("];\n");
}

double QPSolver::unif_controller(double lower, double upper) {
  return lower + ((upper - lower) * rand()) / RAND_MAX;
}

float QPSolver::ran1d_controller(long *idum, int reset) {
  int   j;
  long  k;
  float temp;
  if (reset) {
    iy = 0;
  }
  if (*idum <= 0 || !iy) {
    if (-(*idum) < 1)
      *idum = 1;
    else
      *idum = -(*idum);
    for (j = NTAB + 7; j >= 0; j--) {
      k     = (*idum) / IQ;
      *idum = IA * (*idum - k * IQ) - IR * k;
      if (*idum < 0)
        *idum += IM;
      if (j < NTAB)
        iv[j] = *idum;
    }
    iy = iv[0];
  }
  k     = (*idum) / IQ;
  *idum = IA * (*idum - k * IQ) - IR * k;
  if (*idum < 0)
    *idum += IM;
  j     = iy / NDIV;
  iy    = iv[j];
  iv[j] = *idum;
  if ((temp = AM * iy) > RNMX)
    return RNMX;
  else
    return temp;
}

/* Next function is from numerical recipes in C. */
float QPSolver::randn_internal_controller(long *idum, int reset) {
  float fac, rsq, v1, v2;
  if (reset) {
    iset = 0;
  }
  if (iset == 0) {
    do {
      v1  = 2.0 * ran1d_controller(idum, reset) - 1.0;
      v2  = 2.0 * ran1d_controller(idum, reset) - 1.0;
      rsq = v1 * v1 + v2 * v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac  = sqrt(-2.0 * log(rsq) / rsq);
    gset = v1 * fac;
    iset = 1;
    return v2 * fac;
  } else {
    iset = 0;
    return gset;
  }
}

double QPSolver::randn_controller(void) {
  return randn_internal_controller(&global_seed, 0);
}

void QPSolver::reset_rand_controller(void) {
  srand(15);
  global_seed = 1;
  randn_internal_controller(&global_seed, 1);
}

}  // namespace mpc_controller

}  // namespace mrs_mpc_solvers
