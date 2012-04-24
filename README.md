vfxgal
======

Geometric algorithms library for the VFX industry.

core/
-----
This directory contains the majority of the implementation. Vfxgal is primarily a template-based, generic geometric
algorithm library. It provides functions such as - mesh/plane clipping (with closure); mesh/convex hull clipping;
removal of degenerate mesh vertices; mesh edge addition; and mesh voronoi cell fracturing.

Vfxgal functions use an adaptor pattern, which means that you can write a small adaptor class (see core/adaptors),
and then you will be able to input your own data types directly into the vfxgal algorithms. For eg, at the time of
writing vfxgal ships with houdini adaptors. All you need to do is include core/adaptors/houdini.hpp, and then you
can pass houdini types directly into the various vfxgal algorithms.

houdini/
--------
This is the houdini binding for vfxgal. Various geometric functions are implemented as SOP nodes.

notes
-----
In H11, the voronoi fracturer benchmarked at ~60* faster than houdini's implementation, and is more stable. One 
strange caveat however seemed related to multithreading and the audio driver - the following would result in ~8* 
speed increase on an 8-core machine:
> export LD_PRELOAD=$LD_PRELOAD:/usr/lib64/libaoss.so.0
..which suggested for some reason that mutithreading worked correctly in the presence of libaoss. On a reasonable
8-core machine (can't remember the specs) you should be able to fracture the unit cube into 1M pieces in approx
1 minute. If it's more like 8, you might be having this obscure issue.
