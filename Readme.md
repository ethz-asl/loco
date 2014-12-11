Loco - Locomotion Controller for Legged Robots
-----------------------------------------------------------------
This library provides an interface for a locomotion controller for legged robots.
The current version comes along with an implementation for the quadruped StarlETH (leggedrobotics.ethz.ch).

Autonomous Systems Lab, ETH Zurich

Contact  : Christian Gehring [gehrinch ( at ) ethz.ch]

Author(s): Christian Gehring, Peter Fankhauser, 
           C. Dario Bellicoso, Stelian Coros

Date     : 11-Dec-2014


LICENSE
-----------------------------------------------------------------
This library is Free Software and is licensed under BSD 3-Clause.


DEPENDENCIES
-----------------------------------------------------------------
* Eigen - linear algebra library (eigen.tuxfamily.org)
* kindr - kinematics and dynamics library (http://github.com/ethz-asl/kindr)
* tinyxml - XML parser 
* OOQP - quadratic programming solver (http://pages.cs.wisc.edu/~swright/ooqp/)
* OOQPEI - Eigen interface for OOQP (http://github.com/ethz-asl/ooqp-eigen_interface)
* robotUtils - (trajectories, logger)
* robotModel - (model for robot StarlETH)


INSTALLATION
-----------------------------------------------------------------
Building the library
-----------------------------
Build the library with CMake:
```bash
mkdir build
cd build
cmake ..
make

```

Building and running tests
-----------------------------
GTests are built as soon as the folder gtest exists in the root folder.

Download and use GTest:

```bash
wget http://googletest.googlecode.com/files/gtest-1.7.0.zip
unzip gtest-1.7.0.zip
ln -s gtest-1.7.0 gtest
mkdir build
cd build
cmake .. -DBUILD_TEST=ON
make check
```

Building documentation
-----------------------------

The documentation is generated with Doxygen with:

```bash
cd build
make doc
```

The documentation is then available at `/doc/doxygen/doc/html/index.html`


REFERENCES
-----------------------------------------------------------------
* C. Gehring, S. Coros, M. Hutter, M. Bloesch, M. Hoepflinger, R. Siegwart, “Control of Dynamic Gaits for a Quadrupedal Robot”, IEEE International Conference on Robotics and Automation, 2013.

* C. Gehring, S. Coros, M. Hutter, M. Bloesch, P. Fankhauser, M. A. Hoepflinger, R. Siegwart, “Towards Automatic Discovery of Agile Gaits for Quadrupedal Robots”, Proc. of the IEEE/RSJ IEEE International Conference on Robotics and Automation (ICRA), 2014.
