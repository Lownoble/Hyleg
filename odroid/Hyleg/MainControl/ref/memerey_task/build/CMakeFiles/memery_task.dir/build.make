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
CMAKE_SOURCE_DIR = /home/odroid/MainControl/ref/memerey_task

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/odroid/MainControl/ref/memerey_task/build

# Include any dependencies generated for this target.
include CMakeFiles/memery_task.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/memery_task.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/memery_task.dir/flags.make

CMakeFiles/memery_task.dir/src/comm.c.o: CMakeFiles/memery_task.dir/flags.make
CMakeFiles/memery_task.dir/src/comm.c.o: ../src/comm.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/MainControl/ref/memerey_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/memery_task.dir/src/comm.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/memery_task.dir/src/comm.c.o   -c /home/odroid/MainControl/ref/memerey_task/src/comm.c

CMakeFiles/memery_task.dir/src/comm.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/memery_task.dir/src/comm.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/odroid/MainControl/ref/memerey_task/src/comm.c > CMakeFiles/memery_task.dir/src/comm.c.i

CMakeFiles/memery_task.dir/src/comm.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/memery_task.dir/src/comm.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/odroid/MainControl/ref/memerey_task/src/comm.c -o CMakeFiles/memery_task.dir/src/comm.c.s

CMakeFiles/memery_task.dir/src/main.cpp.o: CMakeFiles/memery_task.dir/flags.make
CMakeFiles/memery_task.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/MainControl/ref/memerey_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/memery_task.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/memery_task.dir/src/main.cpp.o -c /home/odroid/MainControl/ref/memerey_task/src/main.cpp

CMakeFiles/memery_task.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/memery_task.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/MainControl/ref/memerey_task/src/main.cpp > CMakeFiles/memery_task.dir/src/main.cpp.i

CMakeFiles/memery_task.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/memery_task.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/MainControl/ref/memerey_task/src/main.cpp -o CMakeFiles/memery_task.dir/src/main.cpp.s

CMakeFiles/memery_task.dir/src/sys_timer.cpp.o: CMakeFiles/memery_task.dir/flags.make
CMakeFiles/memery_task.dir/src/sys_timer.cpp.o: ../src/sys_timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/odroid/MainControl/ref/memerey_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/memery_task.dir/src/sys_timer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/memery_task.dir/src/sys_timer.cpp.o -c /home/odroid/MainControl/ref/memerey_task/src/sys_timer.cpp

CMakeFiles/memery_task.dir/src/sys_timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/memery_task.dir/src/sys_timer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/odroid/MainControl/ref/memerey_task/src/sys_timer.cpp > CMakeFiles/memery_task.dir/src/sys_timer.cpp.i

CMakeFiles/memery_task.dir/src/sys_timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/memery_task.dir/src/sys_timer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/odroid/MainControl/ref/memerey_task/src/sys_timer.cpp -o CMakeFiles/memery_task.dir/src/sys_timer.cpp.s

# Object files for target memery_task
memery_task_OBJECTS = \
"CMakeFiles/memery_task.dir/src/comm.c.o" \
"CMakeFiles/memery_task.dir/src/main.cpp.o" \
"CMakeFiles/memery_task.dir/src/sys_timer.cpp.o"

# External object files for target memery_task
memery_task_EXTERNAL_OBJECTS =

memery_task: CMakeFiles/memery_task.dir/src/comm.c.o
memery_task: CMakeFiles/memery_task.dir/src/main.cpp.o
memery_task: CMakeFiles/memery_task.dir/src/sys_timer.cpp.o
memery_task: CMakeFiles/memery_task.dir/build.make
memery_task: CMakeFiles/memery_task.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/odroid/MainControl/ref/memerey_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable memery_task"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/memery_task.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/memery_task.dir/build: memery_task

.PHONY : CMakeFiles/memery_task.dir/build

CMakeFiles/memery_task.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/memery_task.dir/cmake_clean.cmake
.PHONY : CMakeFiles/memery_task.dir/clean

CMakeFiles/memery_task.dir/depend:
	cd /home/odroid/MainControl/ref/memerey_task/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/odroid/MainControl/ref/memerey_task /home/odroid/MainControl/ref/memerey_task /home/odroid/MainControl/ref/memerey_task/build /home/odroid/MainControl/ref/memerey_task/build /home/odroid/MainControl/ref/memerey_task/build/CMakeFiles/memery_task.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/memery_task.dir/depend
