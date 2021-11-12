mapMAP MRF MAP Solver [![Build Status](https://travis-ci.org/dthuerck/mapmap_cpu.svg?branch=master)](https://travis-ci.org/dthuerck/mapmap_cpu)
======

Change Log
------
Compared with the development version from our HPG paper (cf. below).

* v1.5 (6/25/2018):
  - Added tech report for new tree selection algorithm (from v1.2) in doc/.
  - Added novel envelopes for supermodular cost function types ("Antipotts",
    "LinearPeak"). A tech report will follow.
  - Several bugfixes.
* v1.4 (1/10/2018):
  - Deterministic solver path with user-provided seed.
  - Several bugfixes and smaller improvements.
* v1.3 (10/18/2017):
  - Envelope optimization for Potts, TruncatedLinear, TruncatedQuadratic.
  - Ability to have individual cost functions per edge.
  - Removed UNARY/PAIRWISE template parameters from solver, hiding these
    internally.
  - Improved multilevel performance, even in the case of individual costs.
  - Added GTest for automatic built instead of a hard dependency.
* v1.2 (5/29/2017):
  - Introduced a new, multicoloring-based tree selection algorithm -
    lock-free.
* v1.1 (4/12/2017):
  - Tuned the tree growing implementation for early termination and
  - option for relaxing the maximality requirement.
* v1.0 (2/8/2017):
  - Stable release.
  - Logging callbacks for use as library.
  - Clean, documented interface and documentation.
  - Added a demo for correct usage.
* beta (12/6/2016):
  - Initial release, mirrors functionality outlined in the paper.
  - Automated vectorization (compile-time detection) for float/double.
  - Supporting scalar/SSE2-4/AVX/AVX2.
  - Added cost function instances.
  - Added unit tests.

Overview
------

CPU-implementation of out massively-parallel, generic, MRF MAP solver named
*mapMAP* posing minimal assumptions to the input, allowing rapid solution
of a large class of MRF problems.

mapMAP's algorithmic foundation and parallelization concept has been presented
at *High Performance Graphics 2016* in Dublin, Ireland. For a reprint and
further information, please refer to our project page (see
below).

Currently, this code implements the following modules and features:

- [x] Customizable (parallel) performance
  - [x] Change cost type between float and double
  - [x] Templated SIMD width (1, 4, 8 for float; 1, 2, 4 for double)
  - [x] Automatically setting SIMD width at compile time
  - [x] Supports SSE4/AVX/AVX2, autodetected during build
  - [x] Automatically using linear-time optimization for certain submodular cost functions
  - [x] Novel linear-time optimization for certain supermodular cost functions
  - [x] Two algorithms for parallel tree sampling
- [x] Extensible interfaces for all components, providing user hooks
  - [x] Cost functions (unary and pairwise)
  - [x] Termination criteria
  - [x] Node grouping criteria for the multilevel module
  - [x] Choice of two (parallel) coordinate selection algorithms
  - [x] Use of heuristics, thereby modifying the solver's structure.
  - [x] User hooks for logging intermediate results.
- [x] Solver modules
  - [x] Acyclic (BCD) descent
  - [x] Spanning tree descent
  - [x] Multilevel solving
- [x] Finding and exploiting connected components in the topology
- [x] Test suite for each individual module

In the future, we will add:
- [ ] Capability to process label costs as outlined in the paper

For the license and terms of usage, please see "License, Terms of usage & Reference".

Prerequisites
------

* CMake building system (>= 3.0.2)
* C++11 compatible compiler (e.g. gcc-5, MSVC 13, icc 17)
* Intel TBB (>= 4.4, see [Webpage](https://www.threadingbuildingblocks.org/))

The code has been tested (and compiles without issues) on an Ubuntu 16.04
system with an AVX-compliant Intel i7-3930K CPU with 64 GB RAM and
using gcc/g++ 5.4.0 and Intel TBB (v2017u3). The latter is
licensed under the 3BSD-compatible Apache 2.0 licence (see
[ASF legal FAQ](http://www.apache.org/legal/resolved.html#category-a)).
Please make sure to use an C++11-comptabile compiler and activate the
necessary options.
If you are a Ubuntu user, please install the packages
`libtbb2 libtbb-dev` (see also Travis CI-script). Google Test will automatically
be downloaded and built.

The provided FindTBB.cmake is taken from [justusc](https://github.com/justusc/FindTBB)
and licensed under the MIT license.

Quickstart
------

The following instructions are provided for linux; the Windows workflow
should be somewhat similar, though GUI-based.

*Step-by-step* instructions:

1. `git clone https://github.com/dthuerck/mapmap_cpu`
2. `cd mapmap_cpu && mkdir build &&  cd build && cmake ..`
3. `ccmake .` and configure the following options (if you want to...):
  * `CMAKE_C_COMPILER` - command for your C-compiler, e.g. `gcc-5`
  * `CMAKE_CXX_COMPILER` - command for your C++-compiler,
    e.g. `g++-5`
  * `TBB_INCLUDE_DIRS` - path containing the `tbb/` folder with
    include files, e.g. `/usr/include`
  * `TBB_LIBRARY` - path containing the TBB library files, e.g.
    `/lib`
  * `BUILD_MEMSAVE` - determines if the dynamic programming should
    allocate memory as needed (`ON`), saving memory but causing
    slightly longer execution times or preallocate the whole table
    (`OFF`)
  * `BUILD_DEMO` - decides whether the demo from the
    wiki is built as `mapmap_demo`
  * `BUILD_TEST` - decides whether the test suite is built as
    `mapmap_test`
4. Configure and generate the Makefile (press `c` and `g` from
   `ccmake`).
5. Build the project using `make` (or `make -j` for parallel build).
6. Depending on your configuration, you can now run `mapmap_test` and/or
   `mapmap_demo` (assuming you activated `BUILD_TEST` and `BUILD_DEMO`).

Using mapMAP as a library in your own projects
------

mapMAP is implemented as a templated, header only library. A simple
```
#include "mapmap/full.h"
```
will do the trick. Remember that in order to work you need to compile your
whole project with C++11 support. All functions and classes are organized
in the namespace ```mapmap```.

For the users of GCC, we recommend the following options for the best
performance:
```
-std=c++11 -Wall -march=native -O2 -flto -mfpmath=sse -funroll-loops
```
As a good starting point, we recommend studying `mapmap_demo.cc` closely,
which is mostly self-explanatory.

Documentation
------

For extended documentation on building, using and extending mapMAP, please
see the
[integrated wiki](https://github.com/dthuerck/mapmap_cpu/wiki).

License, Terms of Usage & Reference
------

Our program is licensed under the liberal BSD 3-Clause license included
as LICENSE.txt file.

If you decide to use our code or code based on this project in your application,
please make sure to cite our HPG 2016 paper:

```
@inproceedings{Thuerck2016MRF,
    title = {A Fast, Massively Parallel Solver for Large, Irregular Pairwise {M}arkov Random Fields},
    author = {Thuerck, Daniel and Waechter, Michael and Widmer, Sven and von Buelow, Max and Seemann, Patrick and Pfetsch, Marc E. and Goesele, Michael},
    booktitle = {Proceedings of High Performance Graphics 2016},
    year = {2016},
}
```
A PDF reprint is available here: [PDF reprint](https://culip.org/files/2016_thuerck_mapmap.pdf) and
[PDF Supplementary Material](https://culip.org/files/2016_thuerck_mapmap_supplemental.pdf).

Contact
------

For any trouble with building, using or extending this software, please use
the project's integrated issue tracker. We'll be happy to help you there or
discuss feature requests.
