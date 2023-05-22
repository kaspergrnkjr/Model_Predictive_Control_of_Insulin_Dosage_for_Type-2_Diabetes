/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fodeeff.h
 *
 * Code generation for function 'fodeeff'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void fodeeff(const emlrtStack *sp, real_T t, const real_T x[73], real_T DgA,
             const real_T param[141], const real_T basal[15], real_T dx[73]);

/* End of code generation (fodeeff.h) */
