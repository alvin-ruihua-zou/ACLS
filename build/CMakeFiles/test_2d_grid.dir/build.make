# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /mnt/c/Users/alvin/Desktop/ACLS

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/Users/alvin/Desktop/ACLS/build

# Include any dependencies generated for this target.
include CMakeFiles/test_2d_grid.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_2d_grid.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_2d_grid.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_2d_grid.dir/flags.make

CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o: ../src/tests/test_2d_grid.cpp
CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/tests/test_2d_grid.cpp

CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/tests/test_2d_grid.cpp > CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.i

CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/tests/test_2d_grid.cpp -o CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.s

CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o: ../src/domains/2dgrid.cpp
CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/domains/2dgrid.cpp

CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/domains/2dgrid.cpp > CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.i

CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/domains/2dgrid.cpp -o CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.s

CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o: ../src/domains/3dgrid.cpp
CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/domains/3dgrid.cpp

CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/domains/3dgrid.cpp > CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.i

CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/domains/3dgrid.cpp -o CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.s

CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o: ../src/domains/tiles.cpp
CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tiles.cpp

CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tiles.cpp > CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.i

CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tiles.cpp -o CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.s

CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o: ../src/domains/tower.cpp
CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tower.cpp

CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tower.cpp > CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.i

CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/domains/tower.cpp -o CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.s

CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o: ../src/planners/ACLS.cpp
CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/planners/ACLS.cpp

CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/planners/ACLS.cpp > CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.i

CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/planners/ACLS.cpp -o CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.s

CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o: ../src/planners/LACLS.cpp
CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/planners/LACLS.cpp

CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/planners/LACLS.cpp > CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.i

CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/planners/LACLS.cpp -o CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.s

CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o: ../src/planners/astar.cpp
CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/planners/astar.cpp

CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/planners/astar.cpp > CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.i

CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/planners/astar.cpp -o CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.s

CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o: ../src/planners/frontier.cpp
CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/planners/frontier.cpp

CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/planners/frontier.cpp > CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.i

CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/planners/frontier.cpp -o CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.s

CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o: ../src/planners/smgs.cpp
CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/planners/smgs.cpp

CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/planners/smgs.cpp > CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.i

CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/planners/smgs.cpp -o CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.s

CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o: ../src/utils/compares.cpp
CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/utils/compares.cpp

CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/utils/compares.cpp > CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.i

CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/utils/compares.cpp -o CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.s

CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o: CMakeFiles/test_2d_grid.dir/flags.make
CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o: ../src/utils/readMap.cpp
CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o: CMakeFiles/test_2d_grid.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o -MF CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o.d -o CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o -c /mnt/c/Users/alvin/Desktop/ACLS/src/utils/readMap.cpp

CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/Users/alvin/Desktop/ACLS/src/utils/readMap.cpp > CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.i

CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/Users/alvin/Desktop/ACLS/src/utils/readMap.cpp -o CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.s

# Object files for target test_2d_grid
test_2d_grid_OBJECTS = \
"CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o" \
"CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o"

# External object files for target test_2d_grid
test_2d_grid_EXTERNAL_OBJECTS =

test_2d_grid: CMakeFiles/test_2d_grid.dir/src/tests/test_2d_grid.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/domains/2dgrid.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/domains/3dgrid.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/domains/tiles.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/domains/tower.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/planners/ACLS.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/planners/LACLS.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/planners/astar.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/planners/frontier.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/planners/smgs.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/utils/compares.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/src/utils/readMap.cpp.o
test_2d_grid: CMakeFiles/test_2d_grid.dir/build.make
test_2d_grid: CMakeFiles/test_2d_grid.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Linking CXX executable test_2d_grid"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_2d_grid.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_2d_grid.dir/build: test_2d_grid
.PHONY : CMakeFiles/test_2d_grid.dir/build

CMakeFiles/test_2d_grid.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_2d_grid.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_2d_grid.dir/clean

CMakeFiles/test_2d_grid.dir/depend:
	cd /mnt/c/Users/alvin/Desktop/ACLS/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/Users/alvin/Desktop/ACLS /mnt/c/Users/alvin/Desktop/ACLS /mnt/c/Users/alvin/Desktop/ACLS/build /mnt/c/Users/alvin/Desktop/ACLS/build /mnt/c/Users/alvin/Desktop/ACLS/build/CMakeFiles/test_2d_grid.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_2d_grid.dir/depend

