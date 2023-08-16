#include <math.h>
#include <stdio.h>

typedef struct Array {
    void* data;
    unsigned long size;
    int sparse;
    const unsigned long* idx;
    unsigned long nnz;
} Array;

struct LangCAtomicFun {
    void* libModel;
    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);
    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

void cartpole_dynamics_flow_map_sparse_jacobian(double const *const * in,
                                                double*const * out,
                                                struct LangCAtomicFun atomicFun) {
   //independent variables
   const double* x = in[0];

   //dependent variables
   double* jac = out[0];

   // auxiliary variables
   double v[10];

   v[0] = 0.1 * cos(x[1]);
   v[1] = 0.146666666666667 - v[0] * v[0];
   v[2] = 1 / v[1];
   v[3] = cos(x[1]);
   v[4] = 0.981 * v[3];
   v[5] = 0.1 * -1 * sin(x[1]);
   v[1] = ((- v[2]) * (-(v[0] * v[5] + v[5] * v[0]))) / v[1];
   v[6] = sin(x[1]);
   v[7] = 0.981 * v[6];
   v[8] = 0 - v[0];
   jac[4] = v[8] * v[2];
   v[9] = 0.1 * x[3] * x[3];
   v[3] = v[9] * v[3];
   v[9] = x[5] + v[9] * v[6];
   jac[2] = 2.2 * v[2] * v[4] + 2.2 * v[1] * v[7] + jac[4] * v[3] + (v[8] * v[1] + (- v[5]) * v[2]) * v[9];
   v[6] = 0.1 * (x[3] + x[3]) * v[6];
   jac[3] = jac[4] * v[6];
   v[0] = 0 - v[0];
   jac[7] = 0.0666666666666667 * v[2];
   jac[5] = v[0] * v[2] * v[4] + (v[0] * v[1] + (- v[5]) * v[2]) * v[7] + jac[7] * v[3] + 0.0666666666666667 * v[1] * v[9];
   jac[6] = jac[7] * v[6];
   // dependent variables without operations
   jac[0] = 1;
   jac[1] = 1;
}

