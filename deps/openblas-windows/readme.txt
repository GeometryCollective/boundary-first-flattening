make QUIET_MAKE=1 TARGET=NEHALEM DYNAMIC_ARCH=1 HOSTCC=gcc NUM_THREADS=64 BINARY=32 CC=i686-w64-mingw32-gcc FC=i686-w64-mingw32-gfortran

 OpenBLAS build complete. (BLAS CBLAS LAPACK LAPACKE)

  OS               ... WINNT             
  Architecture     ... x86               
  BINARY           ... 32bit                 
  C compiler       ... GCC  (command line : i686-w64-mingw32-gcc)
  Fortran compiler ... GFORTRAN  (command line : i686-w64-mingw32-gfortran)
  Library Name     ... libopenblasp-r0.2.15.a (Multi threaded; Max num-threads is 64)

To install the library, you can run "make PREFIX=/path/to/your/installation install".

├── bin
│   └── libopenblas.dll       The shared library for Visual Studio and GCC.
├── include
│   ├── cblas.h
│   ├── f77blas.h
│   ├── lapacke_config.h
│   ├── lapacke.h
│   ├── lapacke_mangling.h
│   ├── lapacke_utils.h
│   └── openblas_config.h
├── lib
│   ├── libopenblas.a         The static library. Only work with GCC.
│   └── libopenblas.dll.a     The import library for Visual Studio.
└── readme.txt
