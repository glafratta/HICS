# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/gula/Desktop/collision-simulation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/gula/Desktop/collision-simulation

# Include any dependencies generated for this target.
include test/CMakeFiles/exampleTrajectories.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/exampleTrajectories.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/exampleTrajectories.dir/flags.make

test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o: test/CMakeFiles/exampleTrajectories.dir/flags.make
test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o: test/makeTrajectories.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/gula/Desktop/collision-simulation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o"
	cd /home/gula/Desktop/collision-simulation/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o -c /home/gula/Desktop/collision-simulation/test/makeTrajectories.cpp

test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.i"
	cd /home/gula/Desktop/collision-simulation/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/gula/Desktop/collision-simulation/test/makeTrajectories.cpp > CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.i

test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.s"
	cd /home/gula/Desktop/collision-simulation/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/gula/Desktop/collision-simulation/test/makeTrajectories.cpp -o CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.s

# Object files for target exampleTrajectories
exampleTrajectories_OBJECTS = \
"CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o"

# External object files for target exampleTrajectories
exampleTrajectories_EXTERNAL_OBJECTS =

test/exampleTrajectories: test/CMakeFiles/exampleTrajectories.dir/makeTrajectories.cpp.o
test/exampleTrajectories: test/CMakeFiles/exampleTrajectories.dir/build.make
test/exampleTrajectories: test/CMakeFiles/exampleTrajectories.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/gula/Desktop/collision-simulation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exampleTrajectories"
	cd /home/gula/Desktop/collision-simulation/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exampleTrajectories.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/exampleTrajectories.dir/build: test/exampleTrajectories

.PHONY : test/CMakeFiles/exampleTrajectories.dir/build

test/CMakeFiles/exampleTrajectories.dir/clean:
	cd /home/gula/Desktop/collision-simulation/test && $(CMAKE_COMMAND) -P CMakeFiles/exampleTrajectories.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/exampleTrajectories.dir/clean

test/CMakeFiles/exampleTrajectories.dir/depend:
	cd /home/gula/Desktop/collision-simulation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/gula/Desktop/collision-simulation /home/gula/Desktop/collision-simulation/test /home/gula/Desktop/collision-simulation /home/gula/Desktop/collision-simulation/test /home/gula/Desktop/collision-simulation/test/CMakeFiles/exampleTrajectories.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/exampleTrajectories.dir/depend

