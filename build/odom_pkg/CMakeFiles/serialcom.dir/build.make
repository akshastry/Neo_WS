# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/su/neo_ws/src/odom_pkg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/su/neo_ws/build/odom_pkg

# Include any dependencies generated for this target.
include CMakeFiles/serialcom.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/serialcom.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/serialcom.dir/flags.make

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o: CMakeFiles/serialcom.dir/flags.make
CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o: /home/su/neo_ws/src/odom_pkg/src/Serial_port_read1.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/su/neo_ws/build/odom_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o -c /home/su/neo_ws/src/odom_pkg/src/Serial_port_read1.cpp

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/su/neo_ws/src/odom_pkg/src/Serial_port_read1.cpp > CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.i

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/su/neo_ws/src/odom_pkg/src/Serial_port_read1.cpp -o CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.s

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.requires:

.PHONY : CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.requires

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.provides: CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.requires
	$(MAKE) -f CMakeFiles/serialcom.dir/build.make CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.provides.build
.PHONY : CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.provides

CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.provides.build: CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o


# Object files for target serialcom
serialcom_OBJECTS = \
"CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o"

# External object files for target serialcom
serialcom_EXTERNAL_OBJECTS =

/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: CMakeFiles/serialcom.dir/build.make
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/libroscpp.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/librosconsole.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/librostime.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /opt/ros/kinetic/lib/libcpp_common.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom: CMakeFiles/serialcom.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/su/neo_ws/build/odom_pkg/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/serialcom.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/serialcom.dir/build: /home/su/neo_ws/devel/.private/odom_pkg/lib/odom_pkg/serialcom

.PHONY : CMakeFiles/serialcom.dir/build

CMakeFiles/serialcom.dir/requires: CMakeFiles/serialcom.dir/src/Serial_port_read1.cpp.o.requires

.PHONY : CMakeFiles/serialcom.dir/requires

CMakeFiles/serialcom.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/serialcom.dir/cmake_clean.cmake
.PHONY : CMakeFiles/serialcom.dir/clean

CMakeFiles/serialcom.dir/depend:
	cd /home/su/neo_ws/build/odom_pkg && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/su/neo_ws/src/odom_pkg /home/su/neo_ws/src/odom_pkg /home/su/neo_ws/build/odom_pkg /home/su/neo_ws/build/odom_pkg /home/su/neo_ws/build/odom_pkg/CMakeFiles/serialcom.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/serialcom.dir/depend

