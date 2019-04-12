#pragma once

#include <nmmintrin.h> //SSE4.2
#include <immintrin.h> //AVX

#define SIMD8

#ifdef SIMD4

#define SIMD_SIZE 4u
#define SimdReg __m128
#define SimdRegi __m128i

#define SET_PS1 _mm_set_ps1
#define LOAD_PS _mm_load_ps
#define LOAD_SI _mm_load_si128
#define STORE_PS _mm_store_ps
#define STORE_I _mm_store_si128
#define SET_ZERO_PS _mm_setzero_ps
#define SET_ZERO_I _mm_setzero_si128
#define SET_I1 _mm_set1_epi32

#define ADD_PS _mm_add_ps
#define SUB_PS _mm_sub_ps
#define MUL_PS _mm_mul_ps
#define DIV_PS _mm_div_ps
#define SQRT_PS _mm_sqrt_ps
#define RSQRT_PS _mm_rsqrt_ps
#define ADD_I _mm_add_epi32
#define SUB_I _mm_sub_epi32

#define CMP_LT_PS _mm_cmplt_ps 
#define CMP_GT_PS _mm_cmpgt_ps 

#define MOVE_MASK_PS _mm_movemask_ps
#define BLEND_PS _mm_blend_ps
#define BLENDV_PS _mm_blendv_ps

#define AND_PS _mm_and_ps
#define NOT_AND_PS _mm_andnot_ps
#define NOT_AND_I _mm_andnot_si128
#define OR_PS _mm_or_ps
#define AND_I _mm_and_si128

#define SL_I _mm_slli_epi32
#define SR_I _mm_srli_epi32

#define MIN_PS _mm_min_ps
#define MAX_PS _mm_max_ps

#define CVT_I_TO_PS _mm_cvtepi32_ps
#define CVT_PS_TO_I _mm_cvtps_epi32

#define CAST_I_TO_PS _mm_castsi128_ps
#define CAST_PS_TO_I _mm_castps_si128

#endif


#ifdef SIMD8

#define SIMD_SIZE 8u
#define SimdReg __m256
#define SimdRegi __m256i


#define SET_PS1 _mm256_set1_ps
#define LOAD_PS _mm256_load_ps
#define LOAD_SI _mm256_load_si256
#define STORE_PS _mm256_store_ps
#define STORE_I _mm256_store_si256
#define SET_ZERO_PS _mm256_setzero_ps
#define SET_ZERO_I _mm256_setzero_si256
#define SET_I1 _mm256_set1_epi32

#define ADD_PS _mm256_add_ps
#define SUB_PS _mm256_sub_ps
#define MUL_PS _mm256_mul_ps
#define DIV_PS _mm256_div_ps
#define SQRT_PS _mm256_sqrt_ps
#define RSQRT_PS _mm256_rsqrt_ps
#define ADD_I _mm256_add_epi32
#define SUB_I _mm256_sub_epi32

#define CMP_LT_PS(a, b) _mm256_cmp_ps(a, b, _CMP_LT_OQ)
#define CMP_GT_PS(a, b) _mm256_cmp_ps(a, b, _CMP_GT_OQ)

#define MOVE_MASK_PS _mm256_movemask_ps
#define BLEND_PS _mm256_blend_ps
#define BLENDV_PS _mm256_blendv_ps

#define AND_PS _mm256_and_ps
#define NOT_AND_PS _mm256_andnot_ps
#define NOT_AND_I _mm256_andnot_si256
#define OR_PS _mm256_or_ps
#define AND_I _mm256_and_si256

#define SL_I _mm256_slli_epi32
#define SR_I _mm256_srli_epi32

#define MIN_PS _mm256_min_ps
#define MAX_PS _mm256_max_ps

#define CVT_I_TO_PS _mm256_cvtepi32_ps
#define CVT_PS_TO_I _mm256_cvtps_epi32

#define CAST_I_TO_PS _mm256_castsi256_ps
#define CAST_PS_TO_I _mm256_castps_si256

#endif

#define PREFETCH_T0(ptr) _mm_prefetch(reinterpret_cast<const char*>(ptr), _MM_HINT_T0)


#include <glm/glm.hpp>
#include "Types.h"

template <byte N>
struct PointPackT {
    float x[N];
    float y[N];
    float z[N];
};

template <byte N>
using VectorPackT = PointPackT<N>;

template <byte N>
using ColorPackT = PointPackT<N>;

template<byte N>
struct RayPackT {
    VectorPackT<N> directions;
    glm::vec3 origin;
};

template <byte N>
struct CollisionPackT {
    PointPackT<N> points;
    float steps[N]; //These are floats but could be ints
};

template <byte N>
struct Point2PackT {
    float x[N];
    float y[N];
};

using PointPack = PointPackT<SIMD_SIZE>;
using RayPack = RayPackT<SIMD_SIZE>;
using CollisionPack = CollisionPackT<SIMD_SIZE>;
using ColorPack = ColorPackT<SIMD_SIZE>;
using VectorPack = VectorPackT<SIMD_SIZE>;
using Point2Pack = Point2PackT<SIMD_SIZE>;

using FloatPack = float[SIMD_SIZE];
using IntPack = int[SIMD_SIZE];


__forceinline SimdReg simdLengthPack(SimdReg x, SimdReg y, SimdReg z) {
    SimdReg xSq = MUL_PS(x, x);
    SimdReg ySq = MUL_PS(y, y);
    SimdReg zSq = MUL_PS(z, z);
    SimdReg sum = ADD_PS(xSq, ADD_PS(ySq, zSq));
    return SQRT_PS(sum);
}

__forceinline SimdReg simdSqLengthPack(SimdReg x, SimdReg y, SimdReg z) {
    SimdReg xSq = MUL_PS(x, x);
    SimdReg ySq = MUL_PS(y, y);
    SimdReg zSq = MUL_PS(z, z);
    return ADD_PS(xSq, ADD_PS(ySq, zSq));
}

__forceinline void simdNormalizePack(SimdReg &x, SimdReg &y, SimdReg &z) {
    SimdReg xSq = MUL_PS(x, x);
    SimdReg ySq = MUL_PS(y, y);
    SimdReg zSq = MUL_PS(z, z);
    SimdReg sum = ADD_PS(xSq, ADD_PS(ySq, zSq));
    SimdReg lengthInv = RSQRT_PS(sum); //1 / sqrt(sum)

    x = MUL_PS(lengthInv, x);
    y = MUL_PS(lengthInv, y);
    z = MUL_PS(lengthInv, z);
}

__forceinline SimdReg simdDotPack(SimdReg x1, SimdReg y1, SimdReg z1, SimdReg x2, SimdReg y2, SimdReg z2) {
    SimdReg x = MUL_PS(x1, x2);
    SimdReg y = MUL_PS(y1, y2);
    SimdReg z = MUL_PS(z1, z2);
    return ADD_PS(x, ADD_PS(y, z));
}

SimdReg simdExp(SimdReg x);
SimdReg simdLog(SimdReg x);

__forceinline SimdReg simdPow(SimdReg x, SimdReg y) {
    return simdExp(MUL_PS(simdLog(x), y));
}


//Place to store constants to go into SimdRegisters (SET_PS1). 
//The compiler would place those constants somewhere on the globals memory, but without any pattern.
//This brings a *big* performance boost on functions that need to load many constants, especially with a smart use of memory prefetching.
extern float SIMD_CONSTANTS[];
void initSimdConstants();