# CPU-RTRM

CPU Real-Time Raymarcher is a renderer based on raymarching distance-fields. It runs exclusively on CPU at interactive frame-rates for relatively simple scenes.

It was a project to learn about SIMD, cache-friendly programming and profiling/optimization.

![](http://www.bmlourenco.com/portfolio/cpu-rtrm/images/list.jpg)

### Demo video: https://youtu.be/sUoEbEefhfQ

## Details
  * Developed with C++.
  * Raymarching based on sphere tracing.
  * Architecture built around SIMD capabilities, using packs of rays. Supports SSE and AVX.
  * Naive multi-threading for the extra speed boost.
  * Implementation of Blinn-Phong lighting, Ambient Occlusion and fog.
