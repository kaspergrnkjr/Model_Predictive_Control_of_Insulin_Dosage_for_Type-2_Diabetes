//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fodeeff_data.cpp
//
// Code generation for function 'fodeeff_data'
//

// Include files
#include "fodeeff_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

boolean_T isMexOutdated;

const char_T *fodeeff_complete_name;

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131626U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "fodeeff",                                            // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

emlrtRSInfo k_emlrtRSI{
    44,                                                               // lineNo
    "mpower",                                                         // fcnName
    "/usr/local/MATLAB/R2022a/toolbox/eml/lib/matlab/matfun/mpower.m" // pathName
};

emlrtRSInfo l_emlrtRSI{
    71,                                                           // lineNo
    "power",                                                      // fcnName
    "/usr/local/MATLAB/R2022a/toolbox/eml/lib/matlab/ops/power.m" // pathName
};

emlrtRTEInfo emlrtRTEI{
    82,                                                           // lineNo
    5,                                                            // colNo
    "fltpower",                                                   // fName
    "/usr/local/MATLAB/R2022a/toolbox/eml/lib/matlab/ops/power.m" // pName
};

// End of code generation (fodeeff_data.cpp)
