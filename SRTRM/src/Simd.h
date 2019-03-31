#pragma once

#include <nmmintrin.h> //SSE4.2
#include <immintrin.h> //AVX

#define SIMD4

#ifdef SIMD4

#define SIMD_SIZE 4u
#define SimdReg __m128
#define SimdRegi __m128i

#define SET_PS1 _mm_set_ps1
#define LOAD_PS _mm_load_ps
#define LOAD_SI _mm_load_si128
#define STORE_PS _mm_store_ps
#define SET_ZERO_PS _mm_setzero_ps

#define ADD_PS _mm_add_ps
#define SUB_PS _mm_sub_ps
#define MUL_PS _mm_mul_ps
#define DIV_PS _mm_div_ps
#define SQRT_PS _mm_sqrt_ps
#define RSQRT_PS _mm_rsqrt_ps

#define CMP_LT_PS _mm_cmplt_ps 
#define CMP_GT_PS _mm_cmpgt_ps 

#define MOVE_MASK_PS _mm_movemask_ps
#define BLEND_PS _mm_blend_ps
#define BLENDV_PS _mm_blendv_ps

#define AND_PS _mm_and_ps
#define NOT_AND_PS _mm_andnot_ps
#define OR_PS _mm_or_ps

#define CVT_I_PS _mm_cvtepi32_ps

#endif


#ifdef SIMD8

#define SIMD_SIZE 8u
#define SimdReg __m256
#define SimdRegi __m256i


#define SET_PS1 _mm256_set1_ps
#define LOAD_PS _mm256_load_ps
#define LOAD_SI _mm256_load_si256
#define STORE_PS _mm256_store_ps
#define SET_ZERO_PS _mm256_setzero_ps

#define ADD_PS _mm256_add_ps
#define SUB_PS _mm256_sub_ps
#define MUL_PS _mm256_mul_ps
#define DIV_PS _mm256_div_ps
#define SQRT_PS _mm256_sqrt_ps
#define RSQRT_PS _mm256_rsqrt_ps

#define CMP_LT_PS(a, b) _mm256_cmp_ps(a, b, _CMP_LT_OQ)
#define CMP_GT_PS(a, b) _mm256_cmp_ps(a, b, _CMP_GT_OQ)

#define MOVE_MASK_PS _mm256_movemask_ps
#define BLEND_PS _mm256_blend_ps
#define BLENDV_PS _mm256_blendv_ps

#define AND_PS _mm256_and_ps
#define NOT_AND_PS _mm256_andnot_ps
#define OR_PS _mm256_or_ps

#define CVT_I_PS _mm256_cvtepi32_ps

#endif


#include <glm/glm.hpp>
#include "Types.h"

template<byte N>
struct RayPackT {
    float dirX[N];
    float dirY[N];
    float dirZ[N];
    glm::vec3 origin;
};

template <byte N>
struct CollisionPackT {
    float pointX[N];
    float pointY[N];
    float pointZ[N];
    float normalX[N];
    float normalY[N];
    float normalZ[N];
};

template <byte N>
struct PointPackT {
    float x[N];
    float y[N];
    float z[N];
};

using RayPack = RayPackT<SIMD_SIZE>;
using CollisionPack = CollisionPackT<SIMD_SIZE>;
using PointPack = PointPackT<SIMD_SIZE>;
using VectorPack = PointPackT<SIMD_SIZE>;
using FloatPack = float[SIMD_SIZE];