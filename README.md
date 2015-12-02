= WOLF - Windowed Localization Frames =

== Overview ==
Wolf is a front-end library to solve localization problems in mobile robotics, such as SLAM, map-based localization, or visual odometry. The approach contemplates the coexistence of multiple sensors, be them synchronized or not. It is thought to build state vectors formed by a set of key-frames (window), and compute error vectors given the available measurements in that window. 

==== Features ====
* State and error mapped to a memory space.
* Multi-sensor
* Different state types
* Highly templatized library

==== Some preliminary documentation ====
* You can visit this  [https://docs.google.com/document/d/1_kBtvCIo33pdP59M3Ib4iEBleDDLcN6yCbmwJDBLtcA Wolf inspiring document]. Contact [mailto:jsola@iri.upc.edu Joan] if you need permissions for the link.
* You can also have a look at the [https://docs.google.com/drawings/d/1jj5VVjQThddswpTPMLG2xv87vtT3o1jiMJo3Mk1Utjg Wolf tree], showing the organization of the main elements in the Wolf project. Contact [mailto:acorominas@iri.upc.edu Andreu] if you need permissions for the link.
* You can finally visit this  [https://docs.google.com/document/d/18XQlgdfTwplakYKKsfw2YAoaVuSyUEQoaU6bYHQDNpU other inspiring document] providing the initial motivation for the Wolf project. Contact [mailto:jsola@iri.upc.edu Joan] if you need permissions for the link.

== Dependencies ==
==== Eigen ====
[http://eigen.tuxfamily.org Eigen]. Linear algebra, header library. 
Eigen 3.2 is also a depencency of ROS-Hydro. In case you don't have ROS in your machine, you can install Eigen by typing:
 $ sudo apt-get install libeigen3-dev

==== Laser Scan Utils ====
'''(1)''' Download:
 $ svn checkout https://devel.iri.upc.edu/labrobotica/algorithms/laser_scan_utils/tags/v1 laser_scan_utils
Or, in case you don't have permissions:
 $ svn checkout https://devel.iri.upc.edu/pub/labrobotica/algorithms/laser_scan_utils/tags/v1 laser_scan_utils

'''(2)''' Build and install:
 $ cd laser_scan_utils/build
 $ cmake ..
 $ make
 $ sudo make install

==== Ceres (5 steps) ====
[http://www.ceres-solver.org/ Ceres] is an optimization library. Currently, this dependency is optional, so the build procedure of Wolf skips part of compilation in case this dependency is not found on the system. '''Installation''' is desctibed at [http://www.ceres-solver.org/building.html Ceres site]. However we report here an alternative step by step procedure to install Ceres.

'''(1)''' Skip this step if Cmake 2.8.0+ and Eigen3.0+ are already installed. Otherwise install them with ''apt-get''.

'''(2) GFLAGS'''
* Git clone the source
 $ git clone https://github.com/gflags/gflags.git

* Build and install with:
 $ cd gflags
 $ mkdir build
 $ cd build
 $ cmake -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" -DGFLAGS_NAMESPACE="google" ..
 $ make
 $ sudo make install 
libgflags.a will be installed at '''/usr/local/lib'''

'''(3) GLOG ''' 
* Download a tar-gz-ball from [https://code.google.com/p/google-glog/ here], download section.
* Uncompress it with: 
 $ tar -xvzf glog-0.3.3.tar.gz 
* Build and install with:
 $ cd glog-0.3.3
 $ ./configure --with-gflags=/usr/local/
 $ make
 $ sudo make install
libglog.so will be installed at '''/usr/local/lib'''

'''(4) SUITESPARSE'''
* Easy way!:
 $ sudo apt-get install libsuitesparse-dev

'''(5) CERES'''
* Get the tar-gz-ball corresponding to the latest stable release from [http://www.ceres-solver.org/building.html here].
* Uncompress it with: 
 $ tar -xvzf ceres-solver-1.10.0.tar.gz
* Build and install with:
 $ cd ceres-solver-1.10.0
 $ mkdir build
 $ cd build
 $ cmake -DCMAKE_CXX_FLAGS="${CMAKE_CXX_FLAGS} -fPIC" ..
 $ make
 $ sudo make install 
libceres.a will be installed at '''/usr/local/lib''' and headers at '''/usr/local/include/ceres'''

== Download and build ==
==== C++ Library ====
'''Download:'''
 $ svn checkout https://devel.iri.upc.edu/labrobotica/algorithms/wolf/tags/v1/ wolf
Or, in case you don't have permissions:
 $ svn checkout https://devel.iri.upc.edu/pub/labrobotica/algorithms/wolf/tags/v1/ wolf

'''Build:'''
 $ cd wolf/build
 $ cmake ..
 $ make
 $ sudo make install  //optional in case you want to install wolf library

==== ROS Node ====
 $ svn checkout https://devel.iri.upc.edu/labrobotica/ros/tags/iri_wolf/v1/
Or, in case you don't have permissions:
 $ svn checkout https://devel.iri.upc.edu/pub/labrobotica/ros/tags/iri_wolf/v1/

== Inspiring Links ==
*[http://www.eventhelix.com/realtimemantra/basics/optimizingcandcppcode.htm Basics on code optimization]

*[http://www.cplusplus.com/forum/articles/10627/ Headers, Includes, Forward declarations, ...]

*Using Eigen quaternion and CERES: [http://www.lloydhughes.co.za/index.php/using-eigen-quaternions-and-ceres-solver/ explanation] & [https://github.com/system123/ceres_extensions GitHub CERES extension]

== Useful tools ==
=== Profiling with Valgrind and Kcachegrind ===
Kcachegrind is a graphical frontend for profiling your program and optimizing your code. 

==== Install in Ubuntu ====
Get the programs with
    sudo apt-get install valgrind kcachegrind

==== Install in Mac OSX ====
In Mac, you can use qcachegrind instead. To get it through Homebrew, type
    brew install valgrind qcachegrind
I don't know if these packages are available through MacPorts. Try
    ports search --name valgrind
    ports search --name qcachegrind
If they are available, just do
    sudo port install valgrind qcachegrind

==== Do the profiling and watch the reports ====
Type in your <code>wolf/bin/</code> directory:
    cd bin/
    valgrind --tool=callgrind ./my_program <my_prg_params>
this produces a log report called <code>callgrind.out.XXXX</code>, where XXXX is a number. Then type (Ubuntu)
    kcachegrind callgrind.out.XXXX
or (Mac)
    qcachegrind callgrind.out.XXXX
and enjoy.