# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/turtlebot/catkin_ws_copy/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/turtlebot/catkin_ws_copy/build

# Include any dependencies generated for this target.
include cmvision/CMakeFiles/colorgui.dir/depend.make

# Include the progress variables for this target.
include cmvision/CMakeFiles/colorgui.dir/progress.make

# Include the compile flags for this target's objects.
include cmvision/CMakeFiles/colorgui.dir/flags.make

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o: cmvision/CMakeFiles/colorgui.dir/flags.make
cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o: /home/turtlebot/catkin_ws_copy/src/cmvision/src/cmvision.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/turtlebot/catkin_ws_copy/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/colorgui.dir/src/cmvision.cc.o -c /home/turtlebot/catkin_ws_copy/src/cmvision/src/cmvision.cc

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colorgui.dir/src/cmvision.cc.i"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/turtlebot/catkin_ws_copy/src/cmvision/src/cmvision.cc > CMakeFiles/colorgui.dir/src/cmvision.cc.i

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colorgui.dir/src/cmvision.cc.s"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/turtlebot/catkin_ws_copy/src/cmvision/src/cmvision.cc -o CMakeFiles/colorgui.dir/src/cmvision.cc.s

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.requires:
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.requires

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.provides: cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.requires
	$(MAKE) -f cmvision/CMakeFiles/colorgui.dir/build.make cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.provides.build
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.provides

cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.provides.build: cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o: cmvision/CMakeFiles/colorgui.dir/flags.make
cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o: /home/turtlebot/catkin_ws_copy/src/cmvision/src/conversions.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/turtlebot/catkin_ws_copy/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/colorgui.dir/src/conversions.c.o   -c /home/turtlebot/catkin_ws_copy/src/cmvision/src/conversions.c

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/colorgui.dir/src/conversions.c.i"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/turtlebot/catkin_ws_copy/src/cmvision/src/conversions.c > CMakeFiles/colorgui.dir/src/conversions.c.i

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/colorgui.dir/src/conversions.c.s"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/turtlebot/catkin_ws_copy/src/cmvision/src/conversions.c -o CMakeFiles/colorgui.dir/src/conversions.c.s

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.requires:
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.requires

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.provides: cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.requires
	$(MAKE) -f cmvision/CMakeFiles/colorgui.dir/build.make cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.provides.build
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.provides

cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.provides.build: cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o: cmvision/CMakeFiles/colorgui.dir/flags.make
cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o: /home/turtlebot/catkin_ws_copy/src/cmvision/src/color_gui.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/turtlebot/catkin_ws_copy/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/colorgui.dir/src/color_gui.cpp.o -c /home/turtlebot/catkin_ws_copy/src/cmvision/src/color_gui.cpp

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/colorgui.dir/src/color_gui.cpp.i"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/turtlebot/catkin_ws_copy/src/cmvision/src/color_gui.cpp > CMakeFiles/colorgui.dir/src/color_gui.cpp.i

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/colorgui.dir/src/color_gui.cpp.s"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/turtlebot/catkin_ws_copy/src/cmvision/src/color_gui.cpp -o CMakeFiles/colorgui.dir/src/color_gui.cpp.s

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.requires:
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.requires

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.provides: cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.requires
	$(MAKE) -f cmvision/CMakeFiles/colorgui.dir/build.make cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.provides.build
.PHONY : cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.provides

cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.provides.build: cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o

# Object files for target colorgui
colorgui_OBJECTS = \
"CMakeFiles/colorgui.dir/src/cmvision.cc.o" \
"CMakeFiles/colorgui.dir/src/conversions.c.o" \
"CMakeFiles/colorgui.dir/src/color_gui.cpp.o"

# External object files for target colorgui
colorgui_EXTERNAL_OBJECTS =

/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: cmvision/CMakeFiles/colorgui.dir/build.make
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/libcv_bridge.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/libroscpp.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/librosconsole.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/liblog4cxx.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/librostime.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /opt/ros/indigo/lib/libcpp_common.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui: cmvision/CMakeFiles/colorgui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui"
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/colorgui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
cmvision/CMakeFiles/colorgui.dir/build: /home/turtlebot/catkin_ws_copy/devel/lib/cmvision/colorgui
.PHONY : cmvision/CMakeFiles/colorgui.dir/build

cmvision/CMakeFiles/colorgui.dir/requires: cmvision/CMakeFiles/colorgui.dir/src/cmvision.cc.o.requires
cmvision/CMakeFiles/colorgui.dir/requires: cmvision/CMakeFiles/colorgui.dir/src/conversions.c.o.requires
cmvision/CMakeFiles/colorgui.dir/requires: cmvision/CMakeFiles/colorgui.dir/src/color_gui.cpp.o.requires
.PHONY : cmvision/CMakeFiles/colorgui.dir/requires

cmvision/CMakeFiles/colorgui.dir/clean:
	cd /home/turtlebot/catkin_ws_copy/build/cmvision && $(CMAKE_COMMAND) -P CMakeFiles/colorgui.dir/cmake_clean.cmake
.PHONY : cmvision/CMakeFiles/colorgui.dir/clean

cmvision/CMakeFiles/colorgui.dir/depend:
	cd /home/turtlebot/catkin_ws_copy/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/turtlebot/catkin_ws_copy/src /home/turtlebot/catkin_ws_copy/src/cmvision /home/turtlebot/catkin_ws_copy/build /home/turtlebot/catkin_ws_copy/build/cmvision /home/turtlebot/catkin_ws_copy/build/cmvision/CMakeFiles/colorgui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : cmvision/CMakeFiles/colorgui.dir/depend

