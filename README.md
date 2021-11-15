TexturingPipeline
--------------------------------------------------------------------------------

Welcome to my project that textures 3D reconstructions from images.
This project focuses on 3D reconstructions generated using structure from
motion and multi-view stereo techniques, however, it is not limited to this
setting.

Dependencies
--------------------------------------------------------------------------------

The code and the build system have the following prerequisites:

- cmake (>= 3.1)
- git
- make
- CGAL (5.2.1)
- openMP
- libtbb (2020U3)
- libpng, libjpg, libtiff
- GMP MPFR Ceres

Compilation ![Build Status](https://travis-ci.org/nmoehrle/mvs-texturing.svg)
--------------------------------------------------------------------------------

1.  `git clone https://github.com/StormPhoenix/TexturingPipeline`
2.  `cd TexturingPipeline`
3.  `mkdir build && cd build && cmake ..`
4.  `make` (or `make -j` for parallel compilation)

References
--------------------------------------------------------------------------------
- [mvs-texturing](https://github.com/nmoehrle/mvs-texturing.git)
- [openMVS](https://github.com/cdcseacave/openMVS)

Contact
--------------------------------------------------------------------------------
https://github.com/StormPhoenix/TexturingPipeline

For further questions you may contact us at
stormphoenix@pku.edu.cn
