# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/asif/Downloads/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/asif/Downloads/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/asif/CLionProjects/DE

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/asif/CLionProjects/DE/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/DE.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DE.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DE.dir/flags.make

CMakeFiles/DE.dir/test/test.cpp.o: CMakeFiles/DE.dir/flags.make
CMakeFiles/DE.dir/test/test.cpp.o: ../test/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/DE.dir/test/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/DE.dir/test/test.cpp.o -c /home/asif/CLionProjects/DE/test/test.cpp

CMakeFiles/DE.dir/test/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DE.dir/test/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/asif/CLionProjects/DE/test/test.cpp > CMakeFiles/DE.dir/test/test.cpp.i

CMakeFiles/DE.dir/test/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DE.dir/test/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/asif/CLionProjects/DE/test/test.cpp -o CMakeFiles/DE.dir/test/test.cpp.s

# Object files for target DE
DE_OBJECTS = \
"CMakeFiles/DE.dir/test/test.cpp.o"

# External object files for target DE
DE_EXTERNAL_OBJECTS =

DE: CMakeFiles/DE.dir/test/test.cpp.o
DE: CMakeFiles/DE.dir/build.make
DE: libmyLib.so
DE: libgl.so
DE: /usr/local/zed/lib/libsl_input.so
DE: /usr/local/zed/lib/libsl_core.so
DE: /usr/local/zed/lib/libsl_zed.so
DE: /usr/lib/libopenblas.so
DE: /usr/lib/x86_64-linux-gnu/libcuda.so
DE: /usr/local/cuda/lib64/libcudart.so
DE: /usr/local/cuda-9.0/lib64/libnppial.so
DE: /usr/local/cuda-9.0/lib64/libnppisu.so
DE: /usr/local/cuda-9.0/lib64/libnppicc.so
DE: /usr/local/cuda-9.0/lib64/libnppicom.so
DE: /usr/local/cuda-9.0/lib64/libnppidei.so
DE: /usr/local/cuda-9.0/lib64/libnppif.so
DE: /usr/local/cuda-9.0/lib64/libnppig.so
DE: /usr/local/cuda-9.0/lib64/libnppim.so
DE: /usr/local/cuda-9.0/lib64/libnppist.so
DE: /usr/local/cuda-9.0/lib64/libnppitc.so
DE: /usr/local/cuda-9.0/lib64/libcublas.so
DE: /usr/local/cuda-9.0/lib64/libcurand.so
DE: /usr/local/cuda-9.0/lib64/libcublas.so
DE: /usr/local/cuda-9.0/lib64/libcurand.so
DE: /usr/local/cuda/lib64/libnpps.so
DE: /usr/lib/x86_64-linux-gnu/libGL.so
DE: /usr/lib/x86_64-linux-gnu/libGLU.so
DE: /usr/lib/x86_64-linux-gnu/libglut.so
DE: /usr/lib/x86_64-linux-gnu/libXmu.so
DE: /usr/lib/x86_64-linux-gnu/libXi.so
DE: /usr/lib/x86_64-linux-gnu/libGLEW.so
DE: /usr/local/lib/libopencv_stitching.so.3.4.7
DE: /usr/local/lib/libopencv_superres.so.3.4.7
DE: /usr/local/lib/libopencv_videostab.so.3.4.7
DE: /usr/local/lib/libopencv_aruco.so.3.4.7
DE: /usr/local/lib/libopencv_bgsegm.so.3.4.7
DE: /usr/local/lib/libopencv_bioinspired.so.3.4.7
DE: /usr/local/lib/libopencv_ccalib.so.3.4.7
DE: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.7
DE: /usr/local/lib/libopencv_dpm.so.3.4.7
DE: /usr/local/lib/libopencv_face.so.3.4.7
DE: /usr/local/lib/libopencv_freetype.so.3.4.7
DE: /usr/local/lib/libopencv_fuzzy.so.3.4.7
DE: /usr/local/lib/libopencv_hfs.so.3.4.7
DE: /usr/local/lib/libopencv_img_hash.so.3.4.7
DE: /usr/local/lib/libopencv_line_descriptor.so.3.4.7
DE: /usr/local/lib/libopencv_optflow.so.3.4.7
DE: /usr/local/lib/libopencv_reg.so.3.4.7
DE: /usr/local/lib/libopencv_rgbd.so.3.4.7
DE: /usr/local/lib/libopencv_saliency.so.3.4.7
DE: /usr/local/lib/libopencv_stereo.so.3.4.7
DE: /usr/local/lib/libopencv_structured_light.so.3.4.7
DE: /usr/local/lib/libopencv_viz.so.3.4.7
DE: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.7
DE: /usr/local/lib/libopencv_surface_matching.so.3.4.7
DE: /usr/local/lib/libopencv_tracking.so.3.4.7
DE: /usr/local/lib/libopencv_datasets.so.3.4.7
DE: /usr/local/lib/libopencv_plot.so.3.4.7
DE: /usr/local/lib/libopencv_text.so.3.4.7
DE: /usr/local/lib/libopencv_dnn.so.3.4.7
DE: /usr/local/lib/libopencv_highgui.so.3.4.7
DE: /usr/local/lib/libopencv_videoio.so.3.4.7
DE: /usr/local/lib/libopencv_xfeatures2d.so.3.4.7
DE: /usr/local/lib/libopencv_ml.so.3.4.7
DE: /usr/local/lib/libopencv_shape.so.3.4.7
DE: /usr/local/lib/libopencv_video.so.3.4.7
DE: /usr/local/lib/libopencv_ximgproc.so.3.4.7
DE: /usr/local/lib/libopencv_xobjdetect.so.3.4.7
DE: /usr/local/lib/libopencv_imgcodecs.so.3.4.7
DE: /usr/local/lib/libopencv_objdetect.so.3.4.7
DE: /usr/local/lib/libopencv_calib3d.so.3.4.7
DE: /usr/local/lib/libopencv_features2d.so.3.4.7
DE: /usr/local/lib/libopencv_flann.so.3.4.7
DE: /usr/local/lib/libopencv_xphoto.so.3.4.7
DE: /usr/local/lib/libopencv_photo.so.3.4.7
DE: /usr/local/lib/libopencv_imgproc.so.3.4.7
DE: /usr/local/lib/libopencv_core.so.3.4.7
DE: CMakeFiles/DE.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable DE"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DE.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DE.dir/build: DE

.PHONY : CMakeFiles/DE.dir/build

CMakeFiles/DE.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DE.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DE.dir/clean

CMakeFiles/DE.dir/depend:
	cd /home/asif/CLionProjects/DE/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/asif/CLionProjects/DE /home/asif/CLionProjects/DE /home/asif/CLionProjects/DE/cmake-build-debug /home/asif/CLionProjects/DE/cmake-build-debug /home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles/DE.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DE.dir/depend

