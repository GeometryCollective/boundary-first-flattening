# boundary-first-flattening

boundary-first-flattening is an optimized implementation of the publication

>    **[Boundary First Flattening](https://arxiv.org/abs/1704.06873)**<br/>
>    [Sawhney](http://www.rohansawhney.io), [Crane](http://www.cs.cmu.edu/~kmcrane/)<br/>
>    ACM Transactions on Graphics

Applications of the Boundary First Flattening algorithm include:
1. Automatic parameterization with optimal area distortion
2. Direct manipulation of boundary lengths or angles
3. Exact preservation of sharp corners
4. Seamless cone parameterization
5. Uniformization over the unit disk
6. Spherical parameterization for genus 0 surfaces

# Screenshot

TODO

# Download Executable

TODO

# Dependencies

1. [SuiteSparse](http://faculty.cse.tamu.edu/davis/suitesparse.html)
2. OpenGL (version 4.1 or higher)
3. [OpenGL Mathematics (GLM)](http://glm.g-truc.net/0.9.8/index.html) (included)
4. [Nanogui](https://github.com/wjakob/nanogui) (included)

# Compiling

On MacOS, compiling should be as simple as

```
git clone https://github.com/rohan-sawhney/boundary-first-flattening.git
cd boundary-first-flattening && git submodule update --init --recursive
mkdir build && cd build && cmake ..
make -j 4
./bff
```

# License

Released under the [MIT License](https://opensource.org/licenses/MIT)
