# Use Gtest
Its a templte project with an example application to show the usage of google test frame work.


In this project a simple implementation for kalman filter as tracker for one object (vehicle) based on sensor data. 

The sensor data are stored in `data` folder in txt format. 

There is parser whic parse data from txt files and feed it to the algorithm
so it can track the object position.

## Important Dependencies
* cmake >= 3.2
  * All OSes: [click here for installation instructions](https://cmake.org/install/)

* gcc/g++ >= 10.3
  * Windows: recommend using [Tdm-gcc](https://jmeubank.github.io/tdm-gcc/download/)

## Basic Build Instructions

1. Clone this repo.
2. Add the google test and Eigen to the environmental path as follow:
  * for googltest `path to\3rd_party\gcc_10_install`
  * for Eigen `path to\3rd_party\Eigen`
3. Make a build directory: `mkdir build && cd build`
4. In the cmake list choose the mode `APP Mode` or `Unit testing Mode` as follow:
  * for `APP Mode` in the main CMakeLists.txt change lines[13,14] as shown:

    option (USE_APP_MODE "Use APP Mode" ON )

    option (USE_UNIT_TESTING "Use Unit testing Mode" OFF)

  * for `Unit testing Mode` in the main CMakeLists.txt change lines[13,14] as shown:

    option (USE_APP_MODE "Use APP Mode" OFF )

    option (USE_UNIT_TESTING "Use Unit testing Mode" ON)

5. generate the make file: `cmake .. -G "MinGW Makefiles"`
6. compile: `path to compiler\TDM-GCC-64\bin\mingw32-make`
7. Run it: `kf.exe` or `gtest.exe`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 4 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.
