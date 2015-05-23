cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

project(HexIII_Safe)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
if(UNIX)
include_directories("/usr/include/Lapack")
include_directories("/usr/include/Aris")
include_directories("/usr/xenomai/include")
include_directories("/opt/etherlab/include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(/usr/lib/Lapack)
link_directories(/usr/lib/Aris)
link_directories(/usr/xenomai/lib)
link_directories(/opt/etherlab/lib)
link_directories(${PCL_LIBRARY_DIRS})
endif(UNIX)
if(WIN32)
include_directories("C:/include/Lapack")
include_directories("C:/include/Aris")
endif(WIN32)

# set link files
set(SYSTEM_LINK_LIBS pthread gfortran)
set(XENOMAI_LINK_LIBS native rtdm xenomai)
set(ETHERLAB_LINK_LIBS ethercat rt)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Control Aris_Core)

# build HexIII_Avoid
set(SOURCE_FILES main.cpp Control.cpp Gait.cpp Server.cpp Hexapod_Robot.cpp Hexapod_Robot_Prmt.cpp)
#PREPEND(FULL_SRC HexIII_Avoid ${SOURCE_FILES})
add_executable(HexIII_Avoid ${SOURCE_FILES})
target_link_libraries(HexIII_Avoid ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${ETHERLAB_LINK_LIBS} ${XENOMAI_LINK_LIBS} ${SYSTEM_LINK_LIBS})

# build Control Client
set(SOURCE_FILES Control_Client.cpp Client.cpp)
#PREPEND(FULL_SRC Control_Client ${SOURCE_FILES})
add_executable(Control_Client ${SOURCE_FILES})
target_link_libraries(Control_Client ${ARIS_LINK_LIBS} ${SYSTEM_LINK_LIBS})

# build programs Vision Clent
set(SOURCE_FILES Vision_Client.cpp Vision_main.cpp Kinect_Test.cpp)
add_executable(Vision_Client ${SOURCE_FILES})
target_link_libraries(Vision_Client ${ARIS_LINK_LIBS}  ${SYSTEM_LINK_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})


