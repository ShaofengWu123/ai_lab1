# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

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
CMAKE_COMMAND = /root/cmake/cmake-3.23.1/bin/cmake

# The command to remove a file.
RM = /root/cmake/cmake-3.23.1/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /root/ai_lab1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/ai_lab1/build

# Include any dependencies generated for this target.
include CMakeFiles/exp1.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/exp1.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/exp1.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/exp1.dir/flags.make

CMakeFiles/exp1.dir/main.cpp.o: CMakeFiles/exp1.dir/flags.make
CMakeFiles/exp1.dir/main.cpp.o: ../main.cpp
CMakeFiles/exp1.dir/main.cpp.o: CMakeFiles/exp1.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/ai_lab1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/exp1.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/exp1.dir/main.cpp.o -MF CMakeFiles/exp1.dir/main.cpp.o.d -o CMakeFiles/exp1.dir/main.cpp.o -c /root/ai_lab1/main.cpp

CMakeFiles/exp1.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/exp1.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/ai_lab1/main.cpp > CMakeFiles/exp1.dir/main.cpp.i

CMakeFiles/exp1.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/exp1.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/ai_lab1/main.cpp -o CMakeFiles/exp1.dir/main.cpp.s

# Object files for target exp1
exp1_OBJECTS = \
"CMakeFiles/exp1.dir/main.cpp.o"

# External object files for target exp1
exp1_EXTERNAL_OBJECTS =

exp1: CMakeFiles/exp1.dir/main.cpp.o
exp1: CMakeFiles/exp1.dir/build.make
exp1: CMakeFiles/exp1.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/ai_lab1/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable exp1"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/exp1.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/exp1.dir/build: exp1
.PHONY : CMakeFiles/exp1.dir/build

CMakeFiles/exp1.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/exp1.dir/cmake_clean.cmake
.PHONY : CMakeFiles/exp1.dir/clean

CMakeFiles/exp1.dir/depend:
	cd /root/ai_lab1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/ai_lab1 /root/ai_lab1 /root/ai_lab1/build /root/ai_lab1/build /root/ai_lab1/build/CMakeFiles/exp1.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/exp1.dir/depend

