# high-level-knitting

A coarse-to-fine design tool for machine knitting.


## Installing

This project uses submodules for some dependencies so, clone with 

```bash
git clone --recursive ...
```

or

```bash
git clone ...
cd stitchgraph
git submodule init
git submodule update
```

### Dependencies
The direct dependencies that must be installed before building are

- libigl
- z3
- OpenMesh
- CGAL
  - boost
- glm

By default, libigl is expected to live as an external project at the same level
as the project. Clone libigl, and run cmake generate on it:

```bash
git clone https://github.com/libigl/libigl.git
cd libigl
mkdir build
cd build
cmake ..
```

#### MacOS

We recommend installing the dependencies with [Homebrew](https://brew.sh):

```bash
brew install z3
brew install open-mesh
brew intall boost
brew install glm
```

#### Windows

We recommend installing the dependencies with
[vcpkg](https://vcpkg.readthedocs.io). We currently only support Arm64 due to
libigl dependencies, so install the x64-windows versions of the packages:

```bash
vcpkg install z3:x64-windows
vcpkg install openmesh:x64-windows
vcpkg install glm:x64-windows
vcpkg install cgal:x64-windows
```

When generating your project files, be sure to give CMake the vcpkg toolchain
file:

```bash
mkdir build
cd build
cmake ..  "-DCMAKE_TOOLCHAIN_FILE=path\to\vcpkg\scripts\buildsystems\vcpkg.cmake"
```
Or, specify the toolchain file in CMake GUI, "Specify toolchain file for cross-
compiling".

#### Linux

Use your distribution's package manager to install z3 and OpenMesh.

### Release Mode

If you want to use the meshing UI, you must compile in release mode. This is because we use CGAL polyhedrons for calculating geodesic paths. This data structure assumes that the meshes will be manifold, but most clothing meshes are not. In release mode, the assertions that check this precondition are not run, and so the code works. In debug mode, the assertions will crash the code immediately upon model import.


### Interaction Operation (1)Remeshing)


Alt+(Middle Button): compute elastic loop
Alt+(Right Buttion): compute geodesic loop
Ctrl+Alt+(Right Buttion):erease face vector (Error!)

Ctrl+(Left Button):add HARD constraint for faces
  1)draw a line on the mesh 
  2)click on two existing vertices
  3)draw a line in a single triangle (Error!)
  
Shift+(Left Button):add SOFT constraint for faces
  1)draw a line on the mesh
  2)click on two existing vertices (Error!)
  
Alt+(Left Button):add seaming lines
  1)draw a line on the mesh (Error!)
  2)click on two existing vertices (Error!)
  







