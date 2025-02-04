# EABMSTLInterface.jl
For interfacing ![EABM.jl](https://github.com/Rufnacous/EABM.jl) with STL 3D model files (ASCII format). Import STLs to describe robot link geometries or export STLs to visualise EABM results in other softwares.

## Import

Use one STL per link geometry, each with one `solid` only. The first line should be comprised as `solid x y z` encoding the coordinates of the segment output. The segment input is the origin, 0 0 0.

## Export

Every articulation properties struct must have an `stl_properties <: STLProperties` attribute. For instance `DesignedSTL <: STLProperties` allows the STL description of that articulation to use an imported STL. Or `PrismaticSTL <: STLProperties`, currently in development, which will be used for writing geometry of multiple articulations as one filamentous component.
