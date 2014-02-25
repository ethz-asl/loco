Loco - Locomotion Controller
-----------------------------------------------------------------
Autonomous Systems Lab
ETH Zurich

Contact  : Christian Gehring [gehrinch ( at ) ethz.ch]

Author(s): Peter Fankhauser, Christian Gehring

Date     : 25-Feb-2014


INSTALLATION
-----------------------------------------------------------------
Building the library:
-----------------------------
Build the library with CMake:
```bash
mkdir build
cd build
cmake ..
make

```

Building google tests
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
