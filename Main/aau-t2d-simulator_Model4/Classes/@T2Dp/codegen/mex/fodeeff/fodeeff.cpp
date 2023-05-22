//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fodeeff.cpp
//
// Code generation for function 'fodeeff'
//

// Include files
#include "fodeeff.h"
#include "fodeeff_data.h"
#include "mpower.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

// Variable Definitions
static emlrtRSInfo emlrtRSI{
    241,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo b_emlrtRSI{
    242,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo c_emlrtRSI{
    243,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo d_emlrtRSI{
    271,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo e_emlrtRSI{
    357,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo f_emlrtRSI{
    358,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo g_emlrtRSI{
    381,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo h_emlrtRSI{
    382,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo i_emlrtRSI{
    388,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

static emlrtRSInfo j_emlrtRSI{
    389,       // lineNo
    "fodeeff", // fcnName
    "/home/maah/Documents/MATLAB/T2D_control/CDC_results/aau-t2d-simulator/"
    "ccodeforode/fodeeff.m" // pathName
};

// Function Definitions
void fodeeff(const emlrtStack *sp, real_T, const real_T x[73], real_T DgA,
             const real_T param[141], const real_T basal[15], real_T dx[73])
{
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack st;
  real_T EGW;
  real_T EL;
  real_T EP;
  real_T Pinft;
  real_T Ra;
  real_T b_dx_tmp;
  real_T c_dx_tmp;
  real_T dx_tmp;
  real_T dx_tmp_tmp;
  real_T gE;
  real_T kempt;
  real_T qslA;
  real_T rKGE;
  real_T rLIC;
  emlrtProfilerSentinel profilerSentinel;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtMEXProfilingFunctionEntryCPP((char_T *)fodeeff_complete_name,
                                    static_cast<boolean_T>(isMexOutdated),
                                    &profilerSentinel);
  // AAU model for simulating a T2D patient.
  // This is the definition of the ode function.
  // X is the state vector
  // Ula is the long-acting mU
  // Ufa is the fast-acting mU
  // Um is the Metaformin oral dose mug
  // Uv is the Vildagliptin oral dose nmol
  // Dg is the amount of oral glucose consumption mg
  // stressv is function in time for stress
  // HRv is a function in time for HR
  // T is a vector of time steps for stress and HR
  // Param is a vector for the parameters of the model
  // Basal is a vector for the basal values
  emlrtMEXProfilingStatement(1, static_cast<boolean_T>(isMexOutdated));
  //  Extract the parameters from the parameter vector:
  // Glucose absorption model:
  emlrtMEXProfilingStatement(3, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(4, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(5, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(6, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(7, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(8, static_cast<boolean_T>(isMexOutdated));
  // Metformin submodel:
  emlrtMEXProfilingStatement(9, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(10, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(11, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(12, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(13, static_cast<boolean_T>(isMexOutdated));
  // Vildagliptin submodel:
  emlrtMEXProfilingStatement(14, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(15, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(16, static_cast<boolean_T>(isMexOutdated));
  // Physical activity model:
  emlrtMEXProfilingStatement(17, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(18, static_cast<boolean_T>(isMexOutdated));
  // Fast acting insulin:
  emlrtMEXProfilingStatement(19, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(20, static_cast<boolean_T>(isMexOutdated));
  // Long acting insulin:
  emlrtMEXProfilingStatement(21, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(22, static_cast<boolean_T>(isMexOutdated));
  // Pancreas submodel
  emlrtMEXProfilingStatement(23, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(24, static_cast<boolean_T>(isMexOutdated));
  // Kpan
  emlrtMEXProfilingStatement(25, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(26, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(27, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(29, static_cast<boolean_T>(isMexOutdated));
  // Insulin submodel:
  emlrtMEXProfilingStatement(30, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(31, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(32, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(33, static_cast<boolean_T>(isMexOutdated));
  // Glucose submodel:
  emlrtMEXProfilingStatement(34, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(35, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(36, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(37, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(38, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(39, static_cast<boolean_T>(isMexOutdated));
  // Glucagon submodel:
  emlrtMEXProfilingStatement(40, static_cast<boolean_T>(isMexOutdated));
  // GLP-1 submodel:
  emlrtMEXProfilingStatement(41, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(42, static_cast<boolean_T>(isMexOutdated));
  // GLP-1 agonists:
  emlrtMEXProfilingStatement(43, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(44, static_cast<boolean_T>(isMexOutdated));
  // rates:
  emlrtMEXProfilingStatement(45, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(46, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(47, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(48, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(49, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(51, static_cast<boolean_T>(isMexOutdated));
  //  Extract states from state vector:
  // Glucose absorption model:
  emlrtMEXProfilingStatement(52, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(53, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(54, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(55, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(56, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(57, static_cast<boolean_T>(isMexOutdated));
  // Metformin submodel:
  emlrtMEXProfilingStatement(58, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(59, static_cast<boolean_T>(isMexOutdated));
  // Vildagliptin submodel:
  emlrtMEXProfilingStatement(60, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(61, static_cast<boolean_T>(isMexOutdated));
  // Physical activity model:
  emlrtMEXProfilingStatement(62, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(63, static_cast<boolean_T>(isMexOutdated));
  // Fast acting insulin:
  emlrtMEXProfilingStatement(64, static_cast<boolean_T>(isMexOutdated));
  // Long acting insulin:
  emlrtMEXProfilingStatement(65, static_cast<boolean_T>(isMexOutdated));
  // Pancreas submodel
  emlrtMEXProfilingStatement(66, static_cast<boolean_T>(isMexOutdated));
  // Insulin submodel:
  emlrtMEXProfilingStatement(67, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(68, static_cast<boolean_T>(isMexOutdated));
  // Glucose submodel:
  emlrtMEXProfilingStatement(69, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(70, static_cast<boolean_T>(isMexOutdated));
  // Glucagon submodel:
  emlrtMEXProfilingStatement(71, static_cast<boolean_T>(isMexOutdated));
  // GLP-1 submodel:
  emlrtMEXProfilingStatement(72, static_cast<boolean_T>(isMexOutdated));
  // GLP-1 agonists
  // Exenatide
  emlrtMEXProfilingStatement(73, static_cast<boolean_T>(isMexOutdated));
  // Semaglutide
  emlrtMEXProfilingStatement(74, static_cast<boolean_T>(isMexOutdated));
  // rates:
  emlrtMEXProfilingStatement(75, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(76, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(77, static_cast<boolean_T>(isMexOutdated));
  // Total glucose consumption:
  // Total Glucose production and appearance:
  // Total insulin consumption:
  // Secreted inuslin:
  // Injected insulin:
  // Integrated hepatic glucose
  //  Basal values and constant rates:
  emlrtMEXProfilingStatement(84, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(85, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(86, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(87, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(88, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(89, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(90, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(91, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(92, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(93, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(94, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(95, static_cast<boolean_T>(isMexOutdated));
  //  Glucose Absorption submodel
  // The model equations
  emlrtMEXProfilingStatement(96, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(97, static_cast<boolean_T>(isMexOutdated));
  qslA = (((x[1] + x[58]) + x[61]) + x[64]) + x[67];
  emlrtMEXProfilingStatement(99, static_cast<boolean_T>(isMexOutdated));
  dx[46] = -param[39] * x[46];
  emlrtMEXProfilingStatement(100, static_cast<boolean_T>(isMexOutdated));
  dx[47] = param[39] * (DgA - x[47]);
  emlrtMEXProfilingStatement(101, static_cast<boolean_T>(isMexOutdated));
  dx[0] = -param[38] * x[0];
  emlrtMEXProfilingStatement(102, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(103, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(104, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(105, static_cast<boolean_T>(isMexOutdated));
  gE = ((((x[0] + x[57]) + x[60]) + x[63]) + x[66]) + qslA;
  kempt = param[39] +
          (param[40] - param[39]) / 2.0 *
              ((muDoubleScalarTanh(5.0 / (2.0 * x[47] * (1.0 - param[36])) *
                                   (gE - param[36] * x[47])) -
                muDoubleScalarTanh(5.0 / (2.0 * x[47] * param[37]) *
                                   (gE - param[37] * x[47]))) +
               2.0);
  emlrtMEXProfilingStatement(106, static_cast<boolean_T>(isMexOutdated));
  dx[1] = -kempt * x[1] + x[0] * param[38];
  emlrtMEXProfilingStatement(107, static_cast<boolean_T>(isMexOutdated));
  dx[2] = -param[41] * x[2] + kempt * x[1];
  // Glucose Absorption (High IG) submodel
  emlrtMEXProfilingStatement(108, static_cast<boolean_T>(isMexOutdated));
  dx[57] = -param[42] * x[57];
  emlrtMEXProfilingStatement(109, static_cast<boolean_T>(isMexOutdated));
  dx[58] = -kempt * x[58] + param[42] * x[57];
  emlrtMEXProfilingStatement(110, static_cast<boolean_T>(isMexOutdated));
  dx[59] = -param[43] * x[59] + kempt * x[58];
  // Glucose Absorption (Medium IG) submodel
  emlrtMEXProfilingStatement(111, static_cast<boolean_T>(isMexOutdated));
  dx[60] = -param[44] * x[60];
  emlrtMEXProfilingStatement(112, static_cast<boolean_T>(isMexOutdated));
  dx[61] = -kempt * x[61] + param[44] * x[60];
  emlrtMEXProfilingStatement(113, static_cast<boolean_T>(isMexOutdated));
  dx[62] = -param[45] * x[62] + kempt * x[61];
  // Glucose Absorption (Low IG) submodel
  emlrtMEXProfilingStatement(114, static_cast<boolean_T>(isMexOutdated));
  dx[63] = -param[46] * x[63];
  emlrtMEXProfilingStatement(115, static_cast<boolean_T>(isMexOutdated));
  dx[64] = -kempt * x[64] + param[46] * x[63];
  emlrtMEXProfilingStatement(116, static_cast<boolean_T>(isMexOutdated));
  dx[65] = -param[47] * x[65] + kempt * x[64];
  // Glucose Absorption (Very low IG) submodel
  emlrtMEXProfilingStatement(117, static_cast<boolean_T>(isMexOutdated));
  dx[66] = -param[48] * x[66];
  emlrtMEXProfilingStatement(118, static_cast<boolean_T>(isMexOutdated));
  dx[67] = -kempt * x[67] + param[48] * x[66];
  emlrtMEXProfilingStatement(119, static_cast<boolean_T>(isMexOutdated));
  dx[68] = -param[49] * x[68] + kempt * x[67];
  emlrtMEXProfilingStatement(120, static_cast<boolean_T>(isMexOutdated));
  Ra = (((param[35] * param[41] * x[2] + param[35] * param[43] * x[59]) +
         param[35] * param[45] * x[62]) +
        param[35] * param[47] * x[65]) +
       param[35] * param[49] * x[68];
  //  Stress:
  // Stress is a parameter that takes values between 1 and 0.
  // It is defined as a vector of a sepcific time step for the simulation.
  // Therefore, it is necessary to interpolate it here:
  emlrtMEXProfilingStatement(121, static_cast<boolean_T>(isMexOutdated));
  // stressv(t);%interp1(0:obj.dt:(length(stressv)*obj.dt-obj.dt),stressv,t);
  // %Interpolates (T,stressv) at time t
  //  Metformin submodel:
  // Model equations:
  emlrtMEXProfilingStatement(122, static_cast<boolean_T>(isMexOutdated));
  dx[3] = -param[117] * x[3];
  emlrtMEXProfilingStatement(123, static_cast<boolean_T>(isMexOutdated));
  dx[4] = -param[118] * x[4];
  emlrtMEXProfilingStatement(124, static_cast<boolean_T>(isMexOutdated));
  dx[5] = (-(param[99] + param[100]) * x[5] + x[3] * param[115]) +
          x[4] * param[116];
  emlrtMEXProfilingStatement(125, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[6] * param[102];
  dx[6] = (x[5] * param[100] + x[8] * param[101]) - dx_tmp;
  emlrtMEXProfilingStatement(126, static_cast<boolean_T>(isMexOutdated));
  b_dx_tmp = x[7] * param[104];
  dx[7] = (dx_tmp + x[8] * param[103]) - b_dx_tmp;
  emlrtMEXProfilingStatement(127, static_cast<boolean_T>(isMexOutdated));
  dx[8] = (b_dx_tmp - ((param[101] + param[103]) + param[105]) * x[8]) + x[5];
  emlrtMEXProfilingStatement(128, static_cast<boolean_T>(isMexOutdated));
  st.site = &emlrtRSI;
  EGW = param[106] * coder::mpower(&st, x[6], param[109]) /
        (coder::mpower(&st, param[112], param[109]) +
         coder::mpower(&st, x[6], param[109]));
  emlrtMEXProfilingStatement(129, static_cast<boolean_T>(isMexOutdated));
  st.site = &b_emlrtRSI;
  EL = param[107] * coder::mpower(&st, x[7], param[110]) /
       (coder::mpower(&st, param[113], param[110]) +
        coder::mpower(&st, x[7], param[110]));
  emlrtMEXProfilingStatement(130, static_cast<boolean_T>(isMexOutdated));
  st.site = &c_emlrtRSI;
  EP = param[108] * coder::mpower(&st, x[8], param[111]) /
       (coder::mpower(&st, param[114], param[111]) +
        coder::mpower(&st, x[8], param[111]));
  //  Vildagliptin submodel:
  // The model equations:
  emlrtMEXProfilingStatement(131, static_cast<boolean_T>(isMexOutdated));
  dx[9] = -param[88] * x[9];
  emlrtMEXProfilingStatement(132, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[10] * param[89];
  dx[10] = x[9] * param[88] - dx_tmp;
  emlrtMEXProfilingStatement(133, static_cast<boolean_T>(isMexOutdated));
  b_dx_tmp = x[11] / param[93];
  dx_tmp_tmp = param[80] - x[13];
  c_dx_tmp = dx_tmp_tmp * param[95] * b_dx_tmp / (param[94] + b_dx_tmp);
  dx[11] = (((dx_tmp - (param[90] + param[91]) / param[93] * x[11]) +
             param[91] / param[92] * x[12]) -
            c_dx_tmp) +
           x[13] * param[96];
  emlrtMEXProfilingStatement(134, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[12] / param[92];
  Pinft = (param[97] - x[14]) * param[95] * dx_tmp / (param[94] + dx_tmp);
  dx[12] = (param[91] * (b_dx_tmp - dx_tmp) - Pinft) + x[14] * param[96];
  emlrtMEXProfilingStatement(135, static_cast<boolean_T>(isMexOutdated));
  dx[13] = c_dx_tmp - (param[96] - param[98]) * x[13];
  emlrtMEXProfilingStatement(136, static_cast<boolean_T>(isMexOutdated));
  dx[14] = Pinft - (param[96] + param[98]) * x[14];
  //  %% GLP-1 agonists
  // (Exenatide):
  emlrtMEXProfilingStatement(137, static_cast<boolean_T>(isMexOutdated));
  dx[69] = -param[81] * x[69];
  emlrtMEXProfilingStatement(138, static_cast<boolean_T>(isMexOutdated));
  dx[70] = param[81] * param[82] * x[69] - x[70] * param[82];
  // (Semaglutide):
  emlrtMEXProfilingStatement(139, static_cast<boolean_T>(isMexOutdated));
  dx[71] = -param[84] * x[71];
  emlrtMEXProfilingStatement(140, static_cast<boolean_T>(isMexOutdated));
  dx[72] = param[84] * param[85] * x[71] - x[72] * param[85];
  //  Physical activity submodel
  emlrtMEXProfilingStatement(141, static_cast<boolean_T>(isMexOutdated));
  // HRv(t);%interp1(0:obj.dt:(length(HRv)*obj.dt-obj.dt),HRv,t); %Interpolates
  // (T,HRv) at time t The model equations:
  emlrtMEXProfilingStatement(142, static_cast<boolean_T>(isMexOutdated));
  dx[15] = 1.0 / param[131] * ((param[137] - param[137]) - x[15]);
  emlrtMEXProfilingStatement(143, static_cast<boolean_T>(isMexOutdated));
  st.site = &d_emlrtRSI;
  gE =
      coder::mpower(&st, x[15] / (param[133] * param[137]), param[132]) /
      (1.0 + coder::mpower(&st, x[15] / (param[133] * param[137]), param[132]));
  emlrtMEXProfilingStatement(144, static_cast<boolean_T>(isMexOutdated));
  dx[48] = 1.0 / param[134] * ((param[138] * gE + param[139]) - x[48]);
  // dx(17) = -(gE+1/TE)*E2+(gE*TE)/(ce1+ce2);
  emlrtMEXProfilingStatement(145, static_cast<boolean_T>(isMexOutdated));
  dx[16] = -(gE + 1.0 / param[134]) * x[16] + gE;
  //  Glucose submodel rates:
  emlrtMEXProfilingStatement(146, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(147, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(148, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(149, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(150, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(151, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(152, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(153, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(154, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(155, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(156, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(157, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(158, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(159, static_cast<boolean_T>(isMexOutdated));
  Pinft = x[36] / basal[3];
  //      if GK>=460
  //          rKGE=330+0.872*GK;
  //      else
  //          rKGE=71+71*tanh(0.011*(GK-460));
  //      end
  emlrtMEXProfilingStatement(160, static_cast<boolean_T>(isMexOutdated));
  rKGE = (0.872 * x[37] + 330.0) *
             (1.0 / (muDoubleScalarExp(-0.05 * (x[37] - 460.0)) + 1.0)) +
         (71.0 * muDoubleScalarTanh(0.011 * (x[37] - 460.0)) + 71.0) *
             (1.0 / (muDoubleScalarExp(0.05 * (x[37] - 460.0)) + 1.0));
  // Effect of Metformin:
  emlrtMEXProfilingStatement(161, static_cast<boolean_T>(isMexOutdated));
  rLIC = 2.7 * muDoubleScalarTanh(0.39 * x[40] / basal[4]);
  emlrtMEXProfilingStatement(162, static_cast<boolean_T>(isMexOutdated));
  gE = basal[11] * (EGW + 1.0);
  emlrtMEXProfilingStatement(163, static_cast<boolean_T>(isMexOutdated));
  //   Rates dynamic model
  emlrtMEXProfilingStatement(164, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[28] / basal[2];
  dx[43] = 0.04 *
           ((1.21 - param[61] * 1.14 *
                        muDoubleScalarTanh(param[51] * (dx_tmp - param[56]))) /
                (1.21 - param[61] * 1.14 *
                            muDoubleScalarTanh(param[51] * (1.0 - param[56]))) -
            x[43]);
  emlrtMEXProfilingStatement(165, static_cast<boolean_T>(isMexOutdated));
  dx[44] = 0.0154 * (0.5 * (rLIC - 1.0) - x[44]);
  emlrtMEXProfilingStatement(166, static_cast<boolean_T>(isMexOutdated));
  dx[45] = 0.04 * (muDoubleScalarTanh(param[53] * (dx_tmp - param[58])) /
                       muDoubleScalarTanh(param[53] * (1.0 - param[58])) -
                   x[45]);
  //   Glucose submodel:
  // +564.4444
  emlrtMEXProfilingStatement(167, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = param[1] / param[15] * (x[32] - x[33]);
  dx[32] = 1.0 / param[0] * (param[8] * (x[34] - x[32]) - dx_tmp);
  emlrtMEXProfilingStatement(168, static_cast<boolean_T>(isMexOutdated));
  dx[33] = 1.0 / param[1] * (dx_tmp - basal[9]);
  emlrtMEXProfilingStatement(169, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = param[11] * x[36];
  dx[34] = 1.0 / param[2] *
           (((((param[8] * x[32] + dx_tmp) + param[13] * x[37]) +
              param[14] * x[38]) -
             param[9] * x[34]) -
            basal[10]);
  emlrtMEXProfilingStatement(170, static_cast<boolean_T>(isMexOutdated));
  dx[35] = 1.0 / param[4] * ((param[12] * (x[34] - x[35]) - gE) + Ra);
  emlrtMEXProfilingStatement(171, static_cast<boolean_T>(isMexOutdated));
  b_dx_tmp = x[16] * param[135];
  c_dx_tmp =
      (b_dx_tmp + 1.0) *
      (x[45] *
       ((5.66 * muDoubleScalarTanh(param[54] * (Pinft - param[59])) + 5.66) /
        (5.66 * muDoubleScalarTanh(param[54] * (1.0 - param[59])) + 5.66)) *
       basal[14]);
  b_dx_tmp =
      (1.0 - b_dx_tmp) *
      (x[43] *
       ((1.42 - 1.41 * muDoubleScalarTanh(param[52] * (Pinft - param[57]))) /
        (1.42 - 1.41 * muDoubleScalarTanh(param[52] * (1.0 - param[57])))) *
       (rLIC - x[44]) * basal[13] * (1.0 - EL));
  dx[36] = 1.0 / param[3] *
           (((param[10] * x[34] + param[12] * x[35]) - dx_tmp) +
            (b_dx_tmp - c_dx_tmp));
  emlrtMEXProfilingStatement(172, static_cast<boolean_T>(isMexOutdated));
  dx[37] = 1.0 / param[5] * (param[13] * (x[34] - x[37]) - rKGE);
  emlrtMEXProfilingStatement(173, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = param[7] / param[16];
  dx[38] =
      1.0 / param[6] * (param[14] * (x[34] - x[38]) - dx_tmp * (x[38] - x[39]));
  emlrtMEXProfilingStatement(174, static_cast<boolean_T>(isMexOutdated));
  Pinft = x[15] * param[136];
  EL = (x[16] * param[135] + 1.0) *
       ((param[62] * 6.52 *
             muDoubleScalarTanh(param[50] * (x[31] / basal[1] - param[55])) +
         7.03) /
        (param[62] * 6.52 * muDoubleScalarTanh(param[50] * (1.0 - param[55])) +
         7.03) *
        (x[39] / basal[0]) * basal[12] * (EP + 1.0));
  dx[39] = 1.0 / param[7] * (dx_tmp * (x[38] - (Pinft + 1.0) * x[39]) - EL);
  emlrtMEXProfilingStatement(175, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(176, static_cast<boolean_T>(isMexOutdated));
  dx[49] = (((((basal[9] + basal[10]) + gE) + c_dx_tmp) + rKGE) +
            Pinft * x[39] * param[14]) +
           EL;
  emlrtMEXProfilingStatement(177, static_cast<boolean_T>(isMexOutdated));
  dx[50] = Ra + b_dx_tmp;
  emlrtMEXProfilingStatement(178, static_cast<boolean_T>(isMexOutdated));
  dx[56] = x[34];
  //  Glucagon submodel:
  // Basal values:
  emlrtMEXProfilingStatement(179, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(180, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(181, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(182, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(183, static_cast<boolean_T>(isMexOutdated));
  dx[40] =
      1.0 / param[34] *
      ((1.31 - 0.61 * muDoubleScalarTanh(1.06 * (x[34] / basal[6] - 0.47))) *
           (2.93 -
            2.09 * muDoubleScalarTanh(4.18 * (x[26] / basal[7] - 0.62))) *
           9.1 -
       9.1 * x[40]);
  //  GLP-1 submodel
  emlrtMEXProfilingStatement(184, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[41] / param[78];
  dx[41] = param[79] * kempt * qslA - dx_tmp;
  emlrtMEXProfilingStatement(185, static_cast<boolean_T>(isMexOutdated));
  dx[42] =
      1.0 / param[75] * (dx_tmp - (param[76] + dx_tmp_tmp * param[77]) * x[42]);
  //  Pancreas submodel
  // The model equations:
  emlrtMEXProfilingStatement(186, static_cast<boolean_T>(isMexOutdated));
  st.site = &e_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  if (x[34] < 0.0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &emlrtRTEI,
                                  "Coder:toolbox:power_domainError",
                                  "Coder:toolbox:power_domainError", 0);
  }
  st.site = &e_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  st.site = &e_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  if (x[34] < 0.0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &emlrtRTEI,
                                  "Coder:toolbox:power_domainError",
                                  "Coder:toolbox:power_domainError", 0);
  }
  gE = muDoubleScalarPower(x[34], 3.27) /
       (5.93 * muDoubleScalarPower(x[34], 3.02) + 2.4790013030205542);
  emlrtMEXProfilingStatement(187, static_cast<boolean_T>(isMexOutdated));
  st.site = &f_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  c_st.site = &l_emlrtRSI;
  if (gE < 0.0) {
    emlrtErrorWithMessageIdR2018a(&c_st, &emlrtRTEI,
                                  "Coder:toolbox:power_domainError",
                                  "Coder:toolbox:power_domainError", 0);
  }
  EL = (x[42] + x[72] * param[86]) + x[70] * param[83];
  Pinft = muDoubleScalarPower(gE, 1.11) + param[63] * EL;
  emlrtMEXProfilingStatement(188, static_cast<boolean_T>(isMexOutdated));
  //      S=mpan*(N1*Y+N2*(XG-R)+zeta2*PSI);
  //      if XG>R
  //          S=Sfactor*mpan*(N1*Y+N2*(XG-R)+zeta2*PSI);
  //      else
  //          S=Sfactor*mpan*(N1*Y+zeta2*PSI);
  //      end
  emlrtMEXProfilingStatement(189, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingStatement(190, static_cast<boolean_T>(isMexOutdated));
  EGW = gE - x[24];
  rLIC = param[71] * Pinft;
  gE = param[64] * EL;
  gE = x[22] * param[74] *
       ((rLIC + gE) * (1.0 / (muDoubleScalarExp(0.5 * EGW) + 1.0)) +
        ((rLIC + param[72] * EGW) + gE) *
            (1.0 / (muDoubleScalarExp(-0.5 * EGW) + 1.0)));
  emlrtMEXProfilingStatement(191, static_cast<boolean_T>(isMexOutdated));
  dx[22] =
      ((param[65] * param[66] - x[22] * param[67]) + x[23] * param[68]) - gE;
  emlrtMEXProfilingStatement(192, static_cast<boolean_T>(isMexOutdated));
  dx[23] = param[69] * (Pinft - x[23]);
  emlrtMEXProfilingStatement(193, static_cast<boolean_T>(isMexOutdated));
  dx[24] = param[70] * EGW;
  //  Insulin submodel rates:
  emlrtMEXProfilingStatement(194, static_cast<boolean_T>(isMexOutdated));
  gE = gE / basal[5] * basal[8];
  emlrtMEXProfilingStatement(195, static_cast<boolean_T>(isMexOutdated));
  EL = param[26] * x[26] + x[27] * param[29];
  rLIC = 0.4 * (EL + gE);
  emlrtMEXProfilingStatement(196, static_cast<boolean_T>(isMexOutdated));
  EGW = 0.3 * param[27] * x[29];
  emlrtMEXProfilingStatement(197, static_cast<boolean_T>(isMexOutdated));
  Pinft = x[31] / (0.85 / (0.15 * param[28]) - 20.0 / param[23]);
  //  Long-acting Insulin:
  // Model equations:
  emlrtMEXProfilingStatement(198, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = param[123] / (x[20] + 1.0);
  dx[19] = -param[124] * x[19] * dx_tmp;
  emlrtMEXProfilingStatement(199, static_cast<boolean_T>(isMexOutdated));
  st.site = &g_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  b_dx_tmp = x[20] - param[121] * muDoubleScalarPower(x[21], 3.0);
  dx[20] = -param[119] * b_dx_tmp + x[19] * param[124] * dx_tmp;
  emlrtMEXProfilingStatement(200, static_cast<boolean_T>(isMexOutdated));
  st.site = &h_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  dx[21] = param[119] * b_dx_tmp - x[21] * param[122] / (x[54] + 1.0);
  emlrtMEXProfilingStatement(201, static_cast<boolean_T>(isMexOutdated));
  dx[54] = param[120] * param[122] * x[21] / (x[54] + 1.0) - x[54] * param[125];
  //  Fast-acting Insulin
  // Model equations:
  emlrtMEXProfilingStatement(202, static_cast<boolean_T>(isMexOutdated));
  st.site = &i_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  dx_tmp = x[17] - param[128] * muDoubleScalarPower(x[18], 3.0);
  dx[17] = -param[126] * dx_tmp;
  emlrtMEXProfilingStatement(203, static_cast<boolean_T>(isMexOutdated));
  st.site = &j_emlrtRSI;
  b_st.site = &k_emlrtRSI;
  dx[18] = param[126] * dx_tmp - x[18] * param[129] / (x[55] + 1.0);
  emlrtMEXProfilingStatement(204, static_cast<boolean_T>(isMexOutdated));
  dx[55] = param[127] * param[129] * x[18] / (x[55] + 1.0) - x[55] * param[130];
  //  Insulin submodel:
  // Model equations:
  emlrtMEXProfilingStatement(205, static_cast<boolean_T>(isMexOutdated));
  dx[25] = param[24] / param[18] * (x[26] - x[25]);
  emlrtMEXProfilingStatement(206, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = x[28] * param[32];
  dx[26] = 1.0 / param[19] *
           ((((param[24] * x[25] + dx_tmp) + param[27] * x[29]) +
             param[28] * x[31]) -
            param[25] * x[26]);
  emlrtMEXProfilingStatement(207, static_cast<boolean_T>(isMexOutdated));
  dx[27] = param[29] / param[20] * (x[26] - x[27]);
  emlrtMEXProfilingStatement(208, static_cast<boolean_T>(isMexOutdated));
  dx[28] = 1.0 / param[21] * (((EL - dx_tmp) + gE) - rLIC);
  emlrtMEXProfilingStatement(209, static_cast<boolean_T>(isMexOutdated));
  dx[29] = 1.0 / param[22] * (param[27] * (x[26] - x[29]) - EGW);
  emlrtMEXProfilingStatement(210, static_cast<boolean_T>(isMexOutdated));
  dx_tmp = param[23] / param[30] * (x[30] - x[31]);
  dx[30] = (1.0 / param[33] * (param[28] * (x[26] - x[30]) - dx_tmp) +
            10.0 * x[55]) +
           10.0 * x[54];
  emlrtMEXProfilingStatement(211, static_cast<boolean_T>(isMexOutdated));
  dx[31] = 1.0 / param[23] * (dx_tmp - Pinft);
  emlrtMEXProfilingStatement(212, static_cast<boolean_T>(isMexOutdated));
  dx[51] = (rLIC + EGW) + Pinft;
  emlrtMEXProfilingStatement(213, static_cast<boolean_T>(isMexOutdated));
  dx[52] = gE;
  emlrtMEXProfilingStatement(214, static_cast<boolean_T>(isMexOutdated));
  dx[53] = param[23] * param[120] * param[122] * x[21] / (x[31] + 1.0) +
           param[23] * param[127] * param[129] * x[18] / (x[31] + 1.0);
  emlrtMEXProfilingStatement(215, static_cast<boolean_T>(isMexOutdated));
  emlrtMEXProfilingFunctionExitCPP(&profilerSentinel);
}

// End of code generation (fodeeff.cpp)
