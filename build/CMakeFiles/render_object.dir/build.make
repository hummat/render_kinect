# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/matthias/.pyenv/versions/3.10.10/envs/legacy/lib/python3.10/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/matthias/.pyenv/versions/3.10.10/envs/legacy/lib/python3.10/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /run/media/matthias/1274B04B74B032F9/git/render_kinect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /run/media/matthias/1274B04B74B032F9/git/render_kinect/build

# Include any dependencies generated for this target.
include CMakeFiles/render_object.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/render_object.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/render_object.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/render_object.dir/flags.make

CMakeFiles/render_object.dir/src/main_kinect.cpp.o: CMakeFiles/render_object.dir/flags.make
CMakeFiles/render_object.dir/src/main_kinect.cpp.o: /run/media/matthias/1274B04B74B032F9/git/render_kinect/src/main_kinect.cpp
CMakeFiles/render_object.dir/src/main_kinect.cpp.o: CMakeFiles/render_object.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/run/media/matthias/1274B04B74B032F9/git/render_kinect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/render_object.dir/src/main_kinect.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/render_object.dir/src/main_kinect.cpp.o -MF CMakeFiles/render_object.dir/src/main_kinect.cpp.o.d -o CMakeFiles/render_object.dir/src/main_kinect.cpp.o -c /run/media/matthias/1274B04B74B032F9/git/render_kinect/src/main_kinect.cpp

CMakeFiles/render_object.dir/src/main_kinect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/render_object.dir/src/main_kinect.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /run/media/matthias/1274B04B74B032F9/git/render_kinect/src/main_kinect.cpp > CMakeFiles/render_object.dir/src/main_kinect.cpp.i

CMakeFiles/render_object.dir/src/main_kinect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/render_object.dir/src/main_kinect.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /run/media/matthias/1274B04B74B032F9/git/render_kinect/src/main_kinect.cpp -o CMakeFiles/render_object.dir/src/main_kinect.cpp.s

# Object files for target render_object
render_object_OBJECTS = \
"CMakeFiles/render_object.dir/src/main_kinect.cpp.o"

# External object files for target render_object
render_object_EXTERNAL_OBJECTS =

/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: CMakeFiles/render_object.dir/src/main_kinect.cpp.o
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: CMakeFiles/render_object.dir/build.make
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /run/media/matthias/1274B04B74B032F9/git/render_kinect/lib/libkinectSim.so
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libgmpxx.so
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libmpfr.so
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libgmp.so
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libopencv_highgui.so.4.7.0
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libopencv_videoio.so.4.7.0
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libopencv_imgcodecs.so.4.7.0
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libopencv_imgproc.so.4.7.0
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: /usr/lib/libopencv_core.so.4.7.0
/run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object: CMakeFiles/render_object.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/run/media/matthias/1274B04B74B032F9/git/render_kinect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/render_object.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/render_object.dir/build: /run/media/matthias/1274B04B74B032F9/git/render_kinect/bin/render_object
.PHONY : CMakeFiles/render_object.dir/build

CMakeFiles/render_object.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/render_object.dir/cmake_clean.cmake
.PHONY : CMakeFiles/render_object.dir/clean

CMakeFiles/render_object.dir/depend:
	cd /run/media/matthias/1274B04B74B032F9/git/render_kinect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /run/media/matthias/1274B04B74B032F9/git/render_kinect /run/media/matthias/1274B04B74B032F9/git/render_kinect /run/media/matthias/1274B04B74B032F9/git/render_kinect/build /run/media/matthias/1274B04B74B032F9/git/render_kinect/build /run/media/matthias/1274B04B74B032F9/git/render_kinect/build/CMakeFiles/render_object.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/render_object.dir/depend

