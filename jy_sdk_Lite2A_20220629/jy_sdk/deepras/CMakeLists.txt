cmake_minimum_required(VERSION 3.4.2)

project(deepras)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11")
set(ROBOT_COMMON "../robot_common")
set(LIB ${ROBOT_COMMON}/lib)
# set(CMAKE_CXX_FLAGS "-O3 -no-pie  -Wall \
# -Wextra -Wcast-align -Wdisabled-optimization -Wformat=2 \
# -Winit-self  -Woverloaded-virtual \
#  -Wshadow -Wsign-promo -std=c++11")
 set(CMAKE_CXX_FLAGS "-O3 -no-pie -w -std=c++11")

## 1. get all source files
set(SRC_DIR_LIST "." motion/WBC_MPC basic_function motion parameters motion/qpOASES motion/qpOASES/include motion/qpOASES/src )

foreach(VAR ${SRC_DIR_LIST})
  set(TEMP)
  aux_source_directory(./src/${VAR} TEMP)
  set(RAS_SRC_LIST ${RAS_SRC_LIST} ${TEMP})
  set(RAS_INC_LIST ${RAS_INC_LIST} ./src/${VAR})
endforeach(VAR)
message(${RAS_SRC_LIST})

## 2. set all include folders


#SET(RAS_SRC_LIST )
#SET(RAS_INC_LIST )
#SET(KPA_INC_LIST )
#SET(SBS_INC_LIST )

#message("${INCLUDE_DIRECTORIES}\n")
include_directories(
  ${LIB}/eigen3
#  ${EIGEN3_INCLUDE_DIR}
  ${LIB}/rcs_lib
  ${RAS_INC_LIST}
 # ${MKL_INCLUDE_DIR}
)


#message("${INCLUDE_DIRECTORIES}\n")

## 3. lib settings
link_directories(
  ${LIB}/rcs_lib/
  ${LIB}/rp_lib/
  ${LIB}/ros_lib/
  ${LIB}/kpa_lib/
  ${LIB}/kpa_lib/linux-uspace/
)


## ?. build setting
add_executable(jy_exe ${RAS_SRC_LIST})
target_link_libraries(jy_exe -ldeeprcs -lrobot_profile -ldeepros)
target_link_libraries(jy_exe -lpthread -lm -lrt -ldl -lstdc++)
#set(EXECUTABLE_OUTPUT_PATH .)

# add_library(deepraslib MODULE ${RAS_SRC_LIST})# forsim

#  add_custom_target(push_to_robot ALL DEPENDS jy_exe)

#  add_custom_command(
#    TARGET push_to_robot POST_BUILD
#    COMMAND ./sftp_to_up.sh
#    WORKING_DIRECTORY ..
#    COMMENT "run get_rcs_lib.sh"
#  )
