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
CMAKE_SOURCE_DIR = /home/chaixun/git/Aris_Control_demo

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chaixun/git/Aris_Control_demo

# Include any dependencies generated for this target.
include CMakeFiles/Control_Client.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Control_Client.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Control_Client.dir/flags.make

CMakeFiles/Control_Client.dir/Control_Client.cpp.o: CMakeFiles/Control_Client.dir/flags.make
CMakeFiles/Control_Client.dir/Control_Client.cpp.o: Control_Client.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chaixun/git/Aris_Control_demo/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Control_Client.dir/Control_Client.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Control_Client.dir/Control_Client.cpp.o -c /home/chaixun/git/Aris_Control_demo/Control_Client.cpp

CMakeFiles/Control_Client.dir/Control_Client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Control_Client.dir/Control_Client.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chaixun/git/Aris_Control_demo/Control_Client.cpp > CMakeFiles/Control_Client.dir/Control_Client.cpp.i

CMakeFiles/Control_Client.dir/Control_Client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Control_Client.dir/Control_Client.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chaixun/git/Aris_Control_demo/Control_Client.cpp -o CMakeFiles/Control_Client.dir/Control_Client.cpp.s

CMakeFiles/Control_Client.dir/Control_Client.cpp.o.requires:
.PHONY : CMakeFiles/Control_Client.dir/Control_Client.cpp.o.requires

CMakeFiles/Control_Client.dir/Control_Client.cpp.o.provides: CMakeFiles/Control_Client.dir/Control_Client.cpp.o.requires
	$(MAKE) -f CMakeFiles/Control_Client.dir/build.make CMakeFiles/Control_Client.dir/Control_Client.cpp.o.provides.build
.PHONY : CMakeFiles/Control_Client.dir/Control_Client.cpp.o.provides

CMakeFiles/Control_Client.dir/Control_Client.cpp.o.provides.build: CMakeFiles/Control_Client.dir/Control_Client.cpp.o

CMakeFiles/Control_Client.dir/Client.cpp.o: CMakeFiles/Control_Client.dir/flags.make
CMakeFiles/Control_Client.dir/Client.cpp.o: Client.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chaixun/git/Aris_Control_demo/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Control_Client.dir/Client.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Control_Client.dir/Client.cpp.o -c /home/chaixun/git/Aris_Control_demo/Client.cpp

CMakeFiles/Control_Client.dir/Client.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Control_Client.dir/Client.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chaixun/git/Aris_Control_demo/Client.cpp > CMakeFiles/Control_Client.dir/Client.cpp.i

CMakeFiles/Control_Client.dir/Client.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Control_Client.dir/Client.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chaixun/git/Aris_Control_demo/Client.cpp -o CMakeFiles/Control_Client.dir/Client.cpp.s

CMakeFiles/Control_Client.dir/Client.cpp.o.requires:
.PHONY : CMakeFiles/Control_Client.dir/Client.cpp.o.requires

CMakeFiles/Control_Client.dir/Client.cpp.o.provides: CMakeFiles/Control_Client.dir/Client.cpp.o.requires
	$(MAKE) -f CMakeFiles/Control_Client.dir/build.make CMakeFiles/Control_Client.dir/Client.cpp.o.provides.build
.PHONY : CMakeFiles/Control_Client.dir/Client.cpp.o.provides

CMakeFiles/Control_Client.dir/Client.cpp.o.provides.build: CMakeFiles/Control_Client.dir/Client.cpp.o

# Object files for target Control_Client
Control_Client_OBJECTS = \
"CMakeFiles/Control_Client.dir/Control_Client.cpp.o" \
"CMakeFiles/Control_Client.dir/Client.cpp.o"

# External object files for target Control_Client
Control_Client_EXTERNAL_OBJECTS =

bin/Control_Client: CMakeFiles/Control_Client.dir/Control_Client.cpp.o
bin/Control_Client: CMakeFiles/Control_Client.dir/Client.cpp.o
bin/Control_Client: CMakeFiles/Control_Client.dir/build.make
bin/Control_Client: CMakeFiles/Control_Client.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable bin/Control_Client"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Control_Client.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Control_Client.dir/build: bin/Control_Client
.PHONY : CMakeFiles/Control_Client.dir/build

CMakeFiles/Control_Client.dir/requires: CMakeFiles/Control_Client.dir/Control_Client.cpp.o.requires
CMakeFiles/Control_Client.dir/requires: CMakeFiles/Control_Client.dir/Client.cpp.o.requires
.PHONY : CMakeFiles/Control_Client.dir/requires

CMakeFiles/Control_Client.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Control_Client.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Control_Client.dir/clean

CMakeFiles/Control_Client.dir/depend:
	cd /home/chaixun/git/Aris_Control_demo && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chaixun/git/Aris_Control_demo /home/chaixun/git/Aris_Control_demo /home/chaixun/git/Aris_Control_demo /home/chaixun/git/Aris_Control_demo /home/chaixun/git/Aris_Control_demo/CMakeFiles/Control_Client.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Control_Client.dir/depend

