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
include CMakeFiles/gl.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gl.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gl.dir/flags.make

CMakeFiles/gl.dir/src/GLViewer.cpp.o: CMakeFiles/gl.dir/flags.make
CMakeFiles/gl.dir/src/GLViewer.cpp.o: ../src/GLViewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gl.dir/src/GLViewer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gl.dir/src/GLViewer.cpp.o -c /home/asif/CLionProjects/DE/src/GLViewer.cpp

CMakeFiles/gl.dir/src/GLViewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gl.dir/src/GLViewer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/asif/CLionProjects/DE/src/GLViewer.cpp > CMakeFiles/gl.dir/src/GLViewer.cpp.i

CMakeFiles/gl.dir/src/GLViewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gl.dir/src/GLViewer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/asif/CLionProjects/DE/src/GLViewer.cpp -o CMakeFiles/gl.dir/src/GLViewer.cpp.s

CMakeFiles/gl.dir/src/birdsEye.cpp.o: CMakeFiles/gl.dir/flags.make
CMakeFiles/gl.dir/src/birdsEye.cpp.o: ../src/birdsEye.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/gl.dir/src/birdsEye.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gl.dir/src/birdsEye.cpp.o -c /home/asif/CLionProjects/DE/src/birdsEye.cpp

CMakeFiles/gl.dir/src/birdsEye.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gl.dir/src/birdsEye.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/asif/CLionProjects/DE/src/birdsEye.cpp > CMakeFiles/gl.dir/src/birdsEye.cpp.i

CMakeFiles/gl.dir/src/birdsEye.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gl.dir/src/birdsEye.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/asif/CLionProjects/DE/src/birdsEye.cpp -o CMakeFiles/gl.dir/src/birdsEye.cpp.s

CMakeFiles/gl.dir/src/pixelcalculato.cpp.o: CMakeFiles/gl.dir/flags.make
CMakeFiles/gl.dir/src/pixelcalculato.cpp.o: ../src/pixelcalculato.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/gl.dir/src/pixelcalculato.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gl.dir/src/pixelcalculato.cpp.o -c /home/asif/CLionProjects/DE/src/pixelcalculato.cpp

CMakeFiles/gl.dir/src/pixelcalculato.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gl.dir/src/pixelcalculato.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/asif/CLionProjects/DE/src/pixelcalculato.cpp > CMakeFiles/gl.dir/src/pixelcalculato.cpp.i

CMakeFiles/gl.dir/src/pixelcalculato.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gl.dir/src/pixelcalculato.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/asif/CLionProjects/DE/src/pixelcalculato.cpp -o CMakeFiles/gl.dir/src/pixelcalculato.cpp.s

# Object files for target gl
gl_OBJECTS = \
"CMakeFiles/gl.dir/src/GLViewer.cpp.o" \
"CMakeFiles/gl.dir/src/birdsEye.cpp.o" \
"CMakeFiles/gl.dir/src/pixelcalculato.cpp.o"

# External object files for target gl
gl_EXTERNAL_OBJECTS =

libgl.so: CMakeFiles/gl.dir/src/GLViewer.cpp.o
libgl.so: CMakeFiles/gl.dir/src/birdsEye.cpp.o
libgl.so: CMakeFiles/gl.dir/src/pixelcalculato.cpp.o
libgl.so: CMakeFiles/gl.dir/build.make
libgl.so: CMakeFiles/gl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library libgl.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gl.dir/build: libgl.so

.PHONY : CMakeFiles/gl.dir/build

CMakeFiles/gl.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gl.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gl.dir/clean

CMakeFiles/gl.dir/depend:
	cd /home/asif/CLionProjects/DE/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/asif/CLionProjects/DE /home/asif/CLionProjects/DE /home/asif/CLionProjects/DE/cmake-build-debug /home/asif/CLionProjects/DE/cmake-build-debug /home/asif/CLionProjects/DE/cmake-build-debug/CMakeFiles/gl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gl.dir/depend

