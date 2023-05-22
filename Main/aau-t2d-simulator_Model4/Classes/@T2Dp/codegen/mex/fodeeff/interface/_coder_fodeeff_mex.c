/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_fodeeff_mex.c
 *
 * Code generation for function '_coder_fodeeff_mex'
 *
 */

/* Include files */
#include "_coder_fodeeff_mex.h"
#include "_coder_fodeeff_api.h"
#include "fodeeff_data.h"
#include "fodeeff_initialize.h"
#include "fodeeff_terminate.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void fodeeff_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
                         const mxArray *prhs[5])
{
  emlrtStack st = {
      NULL, /* site */
      NULL, /* tls */
      NULL  /* prev */
  };
  const mxArray *outputs;
  st.tls = emlrtRootTLSGlobal;
  /* Check for proper number of arguments. */
  if (nrhs != 5) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 5, 4,
                        7, "fodeeff");
  }
  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 7,
                        "fodeeff");
  }
  /* Call the function. */
  fodeeff_api(prhs, &outputs);
  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, &plhs[0], &outputs);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
                 const mxArray *prhs[])
{
  mexAtExit(&fodeeff_atexit);
  /* Module initialization. */
  fodeeff_initialize();
  /* Dispatch the entry-point. */
  fodeeff_mexFunction(nlhs, plhs, nrhs, prhs);
  /* Module termination. */
  fodeeff_terminate();
}

emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLSR2022a(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1,
                           NULL, (const char_T *)"UTF-8", true);
  return emlrtRootTLSGlobal;
}

/* End of code generation (_coder_fodeeff_mex.c) */
