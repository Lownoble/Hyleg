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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/MOCO_ML/control_task

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/MOCO_ML/control_task/build

# Include any dependencies generated for this target.
include CMakeFiles/control_task.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/control_task.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/control_task.dir/flags.make

CMakeFiles/control_task.dir/src/can.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/src/can.cpp.o: ../src/can.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/control_task.dir/src/can.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/src/can.cpp.o -c /home/pi/MOCO_ML/control_task/src/can.cpp

CMakeFiles/control_task.dir/src/can.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/src/can.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/src/can.cpp > CMakeFiles/control_task.dir/src/can.cpp.i

CMakeFiles/control_task.dir/src/can.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/src/can.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/src/can.cpp -o CMakeFiles/control_task.dir/src/can.cpp.s

CMakeFiles/control_task.dir/src/comm.c.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/src/comm.c.o: ../src/comm.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/control_task.dir/src/comm.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/control_task.dir/src/comm.c.o   -c /home/pi/MOCO_ML/control_task/src/comm.c

CMakeFiles/control_task.dir/src/comm.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/control_task.dir/src/comm.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/pi/MOCO_ML/control_task/src/comm.c > CMakeFiles/control_task.dir/src/comm.c.i

CMakeFiles/control_task.dir/src/comm.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/control_task.dir/src/comm.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/pi/MOCO_ML/control_task/src/comm.c -o CMakeFiles/control_task.dir/src/comm.c.s

CMakeFiles/control_task.dir/src/main.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/control_task.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/src/main.cpp.o -c /home/pi/MOCO_ML/control_task/src/main.cpp

CMakeFiles/control_task.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/src/main.cpp > CMakeFiles/control_task.dir/src/main.cpp.i

CMakeFiles/control_task.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/src/main.cpp -o CMakeFiles/control_task.dir/src/main.cpp.s

CMakeFiles/control_task.dir/src/sys_timer.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/src/sys_timer.cpp.o: ../src/sys_timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/control_task.dir/src/sys_timer.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/src/sys_timer.cpp.o -c /home/pi/MOCO_ML/control_task/src/sys_timer.cpp

CMakeFiles/control_task.dir/src/sys_timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/src/sys_timer.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/src/sys_timer.cpp > CMakeFiles/control_task.dir/src/sys_timer.cpp.i

CMakeFiles/control_task.dir/src/sys_timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/src/sys_timer.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/src/sys_timer.cpp -o CMakeFiles/control_task.dir/src/sys_timer.cpp.s

CMakeFiles/control_task.dir/math_src/RT_math.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/RT_math.cpp.o: ../math_src/RT_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/control_task.dir/math_src/RT_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/RT_math.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/RT_math.cpp

CMakeFiles/control_task.dir/math_src/RT_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/RT_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/RT_math.cpp > CMakeFiles/control_task.dir/math_src/RT_math.cpp.i

CMakeFiles/control_task.dir/math_src/RT_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/RT_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/RT_math.cpp -o CMakeFiles/control_task.dir/math_src/RT_math.cpp.s

CMakeFiles/control_task.dir/math_src/common_math.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/common_math.cpp.o: ../math_src/common_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/control_task.dir/math_src/common_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/common_math.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/common_math.cpp

CMakeFiles/control_task.dir/math_src/common_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/common_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/common_math.cpp > CMakeFiles/control_task.dir/math_src/common_math.cpp.i

CMakeFiles/control_task.dir/math_src/common_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/common_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/common_math.cpp -o CMakeFiles/control_task.dir/math_src/common_math.cpp.s

CMakeFiles/control_task.dir/math_src/eso.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/eso.cpp.o: ../math_src/eso.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/control_task.dir/math_src/eso.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/eso.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/eso.cpp

CMakeFiles/control_task.dir/math_src/eso.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/eso.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/eso.cpp > CMakeFiles/control_task.dir/math_src/eso.cpp.i

CMakeFiles/control_task.dir/math_src/eso.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/eso.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/eso.cpp -o CMakeFiles/control_task.dir/math_src/eso.cpp.s

CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o: ../math_src/fliter_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/fliter_math.cpp

CMakeFiles/control_task.dir/math_src/fliter_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/fliter_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/fliter_math.cpp > CMakeFiles/control_task.dir/math_src/fliter_math.cpp.i

CMakeFiles/control_task.dir/math_src/fliter_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/fliter_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/fliter_math.cpp -o CMakeFiles/control_task.dir/math_src/fliter_math.cpp.s

CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o: ../math_src/force_dis8.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/force_dis8.cpp

CMakeFiles/control_task.dir/math_src/force_dis8.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/force_dis8.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/force_dis8.cpp > CMakeFiles/control_task.dir/math_src/force_dis8.cpp.i

CMakeFiles/control_task.dir/math_src/force_dis8.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/force_dis8.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/force_dis8.cpp -o CMakeFiles/control_task.dir/math_src/force_dis8.cpp.s

CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o: ../math_src/ground_att_est_n.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/ground_att_est_n.cpp

CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/ground_att_est_n.cpp > CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.i

CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/ground_att_est_n.cpp -o CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.s

CMakeFiles/control_task.dir/math_src/kin_math.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/kin_math.cpp.o: ../math_src/kin_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/control_task.dir/math_src/kin_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/kin_math.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/kin_math.cpp

CMakeFiles/control_task.dir/math_src/kin_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/kin_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/kin_math.cpp > CMakeFiles/control_task.dir/math_src/kin_math.cpp.i

CMakeFiles/control_task.dir/math_src/kin_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/kin_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/kin_math.cpp -o CMakeFiles/control_task.dir/math_src/kin_math.cpp.s

CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o: ../math_src/leg_planner.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/leg_planner.cpp

CMakeFiles/control_task.dir/math_src/leg_planner.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/leg_planner.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/leg_planner.cpp > CMakeFiles/control_task.dir/math_src/leg_planner.cpp.i

CMakeFiles/control_task.dir/math_src/leg_planner.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/leg_planner.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/leg_planner.cpp -o CMakeFiles/control_task.dir/math_src/leg_planner.cpp.s

CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o: ../math_src/rtGetInf.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/rtGetInf.cpp

CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/rtGetInf.cpp > CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.i

CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/rtGetInf.cpp -o CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.s

CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o: ../math_src/rtGetNaN.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/rtGetNaN.cpp

CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/rtGetNaN.cpp > CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.i

CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/rtGetNaN.cpp -o CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.s

CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o: ../math_src/rt_nonfinite.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/rt_nonfinite.cpp

CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/rt_nonfinite.cpp > CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.i

CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/rt_nonfinite.cpp -o CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.s

CMakeFiles/control_task.dir/math_src/traj_math.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/math_src/traj_math.cpp.o: ../math_src/traj_math.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/control_task.dir/math_src/traj_math.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/math_src/traj_math.cpp.o -c /home/pi/MOCO_ML/control_task/math_src/traj_math.cpp

CMakeFiles/control_task.dir/math_src/traj_math.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/math_src/traj_math.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/math_src/traj_math.cpp > CMakeFiles/control_task.dir/math_src/traj_math.cpp.i

CMakeFiles/control_task.dir/math_src/traj_math.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/math_src/traj_math.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/math_src/traj_math.cpp -o CMakeFiles/control_task.dir/math_src/traj_math.cpp.s

CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o: ../vmc_src/force_imp_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o -c /home/pi/MOCO_ML/control_task/vmc_src/force_imp_controller.cpp

CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/vmc_src/force_imp_controller.cpp > CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.i

CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/vmc_src/force_imp_controller.cpp -o CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.s

CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o: ../vmc_src/hardware_interface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o -c /home/pi/MOCO_ML/control_task/vmc_src/hardware_interface.cpp

CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/vmc_src/hardware_interface.cpp > CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.i

CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/vmc_src/hardware_interface.cpp -o CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.s

CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o: ../vmc_src/locomotion_sfm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o -c /home/pi/MOCO_ML/control_task/vmc_src/locomotion_sfm.cpp

CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/vmc_src/locomotion_sfm.cpp > CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.i

CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/vmc_src/locomotion_sfm.cpp -o CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.s

CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o: ../vmc_src/state_estimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o -c /home/pi/MOCO_ML/control_task/vmc_src/state_estimator.cpp

CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/vmc_src/state_estimator.cpp > CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.i

CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/vmc_src/state_estimator.cpp -o CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.s

CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o: ../vmc_src/vmc_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Building CXX object CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o -c /home/pi/MOCO_ML/control_task/vmc_src/vmc_controller.cpp

CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/vmc_src/vmc_controller.cpp > CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.i

CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/vmc_src/vmc_controller.cpp -o CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.s

CMakeFiles/control_task.dir/gait_src/self_right.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/gait_src/self_right.cpp.o: ../gait_src/self_right.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_22) "Building CXX object CMakeFiles/control_task.dir/gait_src/self_right.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/gait_src/self_right.cpp.o -c /home/pi/MOCO_ML/control_task/gait_src/self_right.cpp

CMakeFiles/control_task.dir/gait_src/self_right.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/gait_src/self_right.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/gait_src/self_right.cpp > CMakeFiles/control_task.dir/gait_src/self_right.cpp.i

CMakeFiles/control_task.dir/gait_src/self_right.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/gait_src/self_right.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/gait_src/self_right.cpp -o CMakeFiles/control_task.dir/gait_src/self_right.cpp.s

CMakeFiles/control_task.dir/gait_src/stand.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/gait_src/stand.cpp.o: ../gait_src/stand.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_23) "Building CXX object CMakeFiles/control_task.dir/gait_src/stand.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/gait_src/stand.cpp.o -c /home/pi/MOCO_ML/control_task/gait_src/stand.cpp

CMakeFiles/control_task.dir/gait_src/stand.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/gait_src/stand.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/gait_src/stand.cpp > CMakeFiles/control_task.dir/gait_src/stand.cpp.i

CMakeFiles/control_task.dir/gait_src/stand.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/gait_src/stand.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/gait_src/stand.cpp -o CMakeFiles/control_task.dir/gait_src/stand.cpp.s

CMakeFiles/control_task.dir/gait_src/tort.cpp.o: CMakeFiles/control_task.dir/flags.make
CMakeFiles/control_task.dir/gait_src/tort.cpp.o: ../gait_src/tort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_24) "Building CXX object CMakeFiles/control_task.dir/gait_src/tort.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/control_task.dir/gait_src/tort.cpp.o -c /home/pi/MOCO_ML/control_task/gait_src/tort.cpp

CMakeFiles/control_task.dir/gait_src/tort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/control_task.dir/gait_src/tort.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/MOCO_ML/control_task/gait_src/tort.cpp > CMakeFiles/control_task.dir/gait_src/tort.cpp.i

CMakeFiles/control_task.dir/gait_src/tort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/control_task.dir/gait_src/tort.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/MOCO_ML/control_task/gait_src/tort.cpp -o CMakeFiles/control_task.dir/gait_src/tort.cpp.s

# Object files for target control_task
control_task_OBJECTS = \
"CMakeFiles/control_task.dir/src/can.cpp.o" \
"CMakeFiles/control_task.dir/src/comm.c.o" \
"CMakeFiles/control_task.dir/src/main.cpp.o" \
"CMakeFiles/control_task.dir/src/sys_timer.cpp.o" \
"CMakeFiles/control_task.dir/math_src/RT_math.cpp.o" \
"CMakeFiles/control_task.dir/math_src/common_math.cpp.o" \
"CMakeFiles/control_task.dir/math_src/eso.cpp.o" \
"CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o" \
"CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o" \
"CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o" \
"CMakeFiles/control_task.dir/math_src/kin_math.cpp.o" \
"CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o" \
"CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o" \
"CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o" \
"CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o" \
"CMakeFiles/control_task.dir/math_src/traj_math.cpp.o" \
"CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o" \
"CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o" \
"CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o" \
"CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o" \
"CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o" \
"CMakeFiles/control_task.dir/gait_src/self_right.cpp.o" \
"CMakeFiles/control_task.dir/gait_src/stand.cpp.o" \
"CMakeFiles/control_task.dir/gait_src/tort.cpp.o"

# External object files for target control_task
control_task_EXTERNAL_OBJECTS =

control_task: CMakeFiles/control_task.dir/src/can.cpp.o
control_task: CMakeFiles/control_task.dir/src/comm.c.o
control_task: CMakeFiles/control_task.dir/src/main.cpp.o
control_task: CMakeFiles/control_task.dir/src/sys_timer.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/RT_math.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/common_math.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/eso.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/fliter_math.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/force_dis8.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/ground_att_est_n.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/kin_math.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/leg_planner.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/rtGetInf.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/rtGetNaN.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/rt_nonfinite.cpp.o
control_task: CMakeFiles/control_task.dir/math_src/traj_math.cpp.o
control_task: CMakeFiles/control_task.dir/vmc_src/force_imp_controller.cpp.o
control_task: CMakeFiles/control_task.dir/vmc_src/hardware_interface.cpp.o
control_task: CMakeFiles/control_task.dir/vmc_src/locomotion_sfm.cpp.o
control_task: CMakeFiles/control_task.dir/vmc_src/state_estimator.cpp.o
control_task: CMakeFiles/control_task.dir/vmc_src/vmc_controller.cpp.o
control_task: CMakeFiles/control_task.dir/gait_src/self_right.cpp.o
control_task: CMakeFiles/control_task.dir/gait_src/stand.cpp.o
control_task: CMakeFiles/control_task.dir/gait_src/tort.cpp.o
control_task: CMakeFiles/control_task.dir/build.make
control_task: CMakeFiles/control_task.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/MOCO_ML/control_task/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_25) "Linking CXX executable control_task"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/control_task.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/control_task.dir/build: control_task

.PHONY : CMakeFiles/control_task.dir/build

CMakeFiles/control_task.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/control_task.dir/cmake_clean.cmake
.PHONY : CMakeFiles/control_task.dir/clean

CMakeFiles/control_task.dir/depend:
	cd /home/pi/MOCO_ML/control_task/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/MOCO_ML/control_task /home/pi/MOCO_ML/control_task /home/pi/MOCO_ML/control_task/build /home/pi/MOCO_ML/control_task/build /home/pi/MOCO_ML/control_task/build/CMakeFiles/control_task.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/control_task.dir/depend

