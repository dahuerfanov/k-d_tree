## k-dimensional tree

In this task, you will implement a nearest-neighbour search in euclidean space using a k-d tree.

1. Implement the remaining part of the nearestPointInSubtree function (see instructions in the code).
2. Create a simple project based on main.cpp that features some kind of a build config (make or cmake, Linux environment).

## Use the project

### Requisites

The following tools must be installed in your Linux system:

1. The GCC compiler
2. CMake `>=3.10`

### Build the project

Once the repo is cloned in your machine, go to the repo root directory and run:

1. `mkdir -p build`
2. `cd build`
3. `cmake ..`
4. `cmake --build .`

The executable will be generated in the `build` directory. Run it with `./kd_tree`.