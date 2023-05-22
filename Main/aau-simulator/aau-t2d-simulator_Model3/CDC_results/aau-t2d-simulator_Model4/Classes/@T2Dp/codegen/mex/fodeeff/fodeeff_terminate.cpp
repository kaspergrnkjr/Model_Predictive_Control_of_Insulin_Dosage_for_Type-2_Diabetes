//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fodeeff_terminate.cpp
//
// Code generation for function 'fodeeff_terminate'
//

// Include files
#include "fodeeff_terminate.h"
#include "_coder_fodeeff_mex.h"
#include "fodeeff_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void fodeeff_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtProfilerUnregisterMEXFcn((char_T *)fodeeff_complete_name,
                                static_cast<boolean_T>(isMexOutdated));
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void fodeeff_terminate()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (fodeeff_terminate.cpp)
