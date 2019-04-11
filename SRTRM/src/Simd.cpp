#include "Simd.h"

#define EXP_POLY_DEGREE 2

#define POLY0(x, c0) SET_PS1(c0)
#define POLY1(x, c0, c1) ADD_PS(MUL_PS(POLY0(x, c1), x), SET_PS1(c0))
#define POLY2(x, c0, c1, c2) ADD_PS(MUL_PS(POLY1(x, c1, c2), x), SET_PS1(c0))
#define POLY3(x, c0, c1, c2, c3) ADD_PS(MUL_PS(POLY2(x, c1, c2, c3), x), SET_PS1(c0))
#define POLY4(x, c0, c1, c2, c3, c4) ADD_PS(MUL_PS(POLY3(x, c1, c2, c3, c4), x), SET_PS1(c0))
#define POLY5(x, c0, c1, c2, c3, c4, c5) ADD_PS(MUL_PS(POLY4(x, c1, c2, c3, c4, c5), x), SET_PS1(c0))

SimdReg simdExp(SimdReg x) {
    SimdRegi ipart;
    SimdReg fpart, expipart, expfpart;

    x = MIN_PS(x, SET_PS1(129.00000f));
    x = MAX_PS(x, SET_PS1(-126.99999f));

    /* ipart = int(x - 0.5) */
    ipart = CVT_PS_TO_I(SUB_PS(x, SET_PS1(0.5f)));

    /* fpart = x - ipart */
    fpart = SUB_PS(x, CVT_I_TO_PS(ipart));

    /* expipart = (float) (1 << ipart) */
    expipart = CAST_I_TO_PS(SL_I(ADD_I(ipart, SET_I1(127)), 23));

    /* minimax polynomial fit of 2**x, in range [-0.5, 0.5[ */
#if EXP_POLY_DEGREE == 5
    expfpart = POLY5(fpart, 9.9999994e-1f, 6.9315308e-1f, 2.4015361e-1f, 5.5826318e-2f, 8.9893397e-3f, 1.8775767e-3f);
#elif EXP_POLY_DEGREE == 4
    expfpart = POLY4(fpart, 1.0000026f, 6.9300383e-1f, 2.4144275e-1f, 5.2011464e-2f, 1.3534167e-2f);
#elif EXP_POLY_DEGREE == 3
    expfpart = POLY3(fpart, 9.9992520e-1f, 6.9583356e-1f, 2.2606716e-1f, 7.8024521e-2f);
#elif EXP_POLY_DEGREE == 2
    expfpart = POLY2(fpart, 1.0017247f, 6.5763628e-1f, 3.3718944e-1f);
#else
#error
#endif

    return MUL_PS(expipart, expfpart);
}


#define LOG_POLY_DEGREE 3

SimdReg simdLog(SimdReg x) {
    SimdRegi exp = SET_I1(0x7F800000);
    SimdRegi mant = SET_I1(0x007FFFFF);

    SimdReg one = SET_PS1(1.0f);
    SimdRegi i = CAST_PS_TO_I(x);

    SimdReg e = CVT_I_TO_PS(SUB_I(SR_I(AND_I(i, exp), 23), SET_I1(127)));

    SimdReg m = OR_PS(CAST_I_TO_PS(AND_I(i, mant)), one);

    SimdReg p;

    /* Minimax polynomial fit of log2(x)/(x - 1), for x in range [1, 2[ */
#if LOG_POLY_DEGREE == 6
    p = POLY5(m, 3.1157899f, -3.3241990f, 2.5988452f, -1.2315303f, 3.1821337e-1f, -3.4436006e-2f);
#elif LOG_POLY_DEGREE == 5
    p = POLY4(m, 2.8882704548164776201f, -2.52074962577807006663f, 1.48116647521213171641f, -0.465725644288844778798f, 0.0596515482674574969533f);
#elif LOG_POLY_DEGREE == 4
    p = POLY3(m, 2.61761038894603480148f, -1.75647175389045657003f, 0.688243882994381274313f, -0.107254423828329604454f);
#elif LOG_POLY_DEGREE == 3
    p = POLY2(m, 2.28330284476918490682f, -1.04913055217340124191f, 0.204446009836232697516f);
#else
#error
#endif

   /* This effectively increases the polynomial degree by one, but ensures that log2(1) == 0*/
    p = MUL_PS(p, SUB_PS(m, one));
    return ADD_PS(p, e);
}


float SIMD_CONSTANTS[SIMD_SIZE * 4];

void initSimdConstants() {
    const float VALUES[] = {-1.0f, 1.0f, 2.0f, 0.57735026918962576450914878050196f /*1 / sqrt(3)*/};

    for(int i = 0; i < sizeof(VALUES) / sizeof(float); ++i) {
        for(int j = 0; j < SIMD_SIZE; ++j) {
            SIMD_CONSTANTS[i * SIMD_SIZE + j] = VALUES[i];
        }
    }
}