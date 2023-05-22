//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fodeeff_initialize.cpp
//
// Code generation for function 'fodeeff_initialize'
//

// Include files
#include "fodeeff_initialize.h"
#include "_coder_fodeeff_mex.h"
#include "fodeeff_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void fodeeff_once();

// Function Definitions
static void fodeeff_once()
{
  static const int32_T lineInfo[215]{
      15,  16,  21,  22,  24,  26,  28,  30,  34,  35,  36,  37,  38,  41,  42,
      43,  46,  47,  50,  51,  54,  55,  58,  59,  60,  61,  62,  63,  64,  67,
      68,  69,  70,  74,  75,  76,  77,  78,  79,  82,  86,  87,  90,  91,  94,
      95,  96,  97,  98,  99,  100, 104, 105, 106, 107, 108, 109, 112, 113, 116,
      117, 120, 121, 123, 126, 129, 132, 133, 136, 137, 140, 143, 148, 151, 154,
      155, 156, 159, 161, 163, 165, 167, 170, 172, 173, 175, 176, 177, 178, 179,
      180, 181, 182, 183, 184, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197,
      198, 199, 202, 203, 204, 207, 208, 209, 212, 213, 214, 217, 218, 219, 222,
      228, 233, 234, 235, 236, 237, 238, 241, 242, 243, 247, 248, 249, 250, 251,
      252, 259, 260, 264, 265, 268, 270, 271, 272, 274, 278, 280, 282, 285, 286,
      288, 289, 291, 293, 295, 296, 298, 299, 301, 308, 310, 311, 312, 315, 316,
      317, 321, 322, 323, 324, 325, 326, 327, 328, 331, 332, 333, 334, 339, 341,
      342, 344, 346, 350, 351, 357, 358, 359, 366, 367, 368, 369, 370, 373, 374,
      375, 376, 380, 381, 382, 383, 388, 389, 390, 394, 395, 396, 397, 398, 399,
      400, 402, 403, 404, 405};
  mex_InitInfAndNan();
  fodeeff_complete_name =
      "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
      "ccodeforode/fodeeff.m>fodeeff(codegen)";
  isMexOutdated = emlrtProfilerCheckMEXOutdated();
  emlrtProfilerRegisterMEXFcn(
      (char_T *)fodeeff_complete_name,
      (char_T *)"/home/maah/Documents/MATLAB/T2D_control/CDC_results/"
                "aau-t2d-simulator/ccodeforode/fodeeff.m",
      (char_T *)"fodeeff", 215, &lineInfo[0],
      static_cast<boolean_T>(isMexOutdated));
}

void fodeeff_initialize()
{
  static const volatile char_T *emlrtBreakCheckR2012bFlagVar{nullptr};
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    fodeeff_once();
  }
  emlrtCheckProfilerStatus();
}

// End of code generation (fodeeff_initialize.cpp)
