# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "px_comm: 3 messages, 1 services")

set(MSG_I_FLAGS "-Ipx_comm:/home/su/neo_ws/src/px-ros-pkg/px_comm/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(px_comm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_custom_target(_px_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "px_comm" "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_custom_target(_px_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "px_comm" "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" "px_comm/CameraInfo:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_custom_target(_px_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "px_comm" "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_custom_target(_px_comm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "px_comm" "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
)
_generate_msg_cpp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
)
_generate_msg_cpp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
)

### Generating Services
_generate_srv_cpp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
)

### Generating Module File
_generate_module_cpp(px_comm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(px_comm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(px_comm_generate_messages px_comm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_cpp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_dependencies(px_comm_generate_messages_cpp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_cpp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_cpp _px_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px_comm_gencpp)
add_dependencies(px_comm_gencpp px_comm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px_comm_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
)
_generate_msg_eus(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
)
_generate_msg_eus(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
)

### Generating Services
_generate_srv_eus(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
)

### Generating Module File
_generate_module_eus(px_comm
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(px_comm_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(px_comm_generate_messages px_comm_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_eus _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_dependencies(px_comm_generate_messages_eus _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_eus _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_eus _px_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px_comm_geneus)
add_dependencies(px_comm_geneus px_comm_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px_comm_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
)
_generate_msg_lisp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
)
_generate_msg_lisp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
)

### Generating Services
_generate_srv_lisp(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
)

### Generating Module File
_generate_module_lisp(px_comm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(px_comm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(px_comm_generate_messages px_comm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_lisp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_dependencies(px_comm_generate_messages_lisp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_lisp _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_lisp _px_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px_comm_genlisp)
add_dependencies(px_comm_genlisp px_comm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px_comm_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
)
_generate_msg_nodejs(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
)
_generate_msg_nodejs(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
)

### Generating Services
_generate_srv_nodejs(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
)

### Generating Module File
_generate_module_nodejs(px_comm
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(px_comm_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(px_comm_generate_messages px_comm_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_nodejs _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_dependencies(px_comm_generate_messages_nodejs _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_nodejs _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_nodejs _px_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px_comm_gennodejs)
add_dependencies(px_comm_gennodejs px_comm_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px_comm_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
)
_generate_msg_py(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
)
_generate_msg_py(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
)

### Generating Services
_generate_srv_py(px_comm
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv"
  "${MSG_I_FLAGS}"
  "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
)

### Generating Module File
_generate_module_py(px_comm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(px_comm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(px_comm_generate_messages px_comm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/Mavlink.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_py _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/srv/SetCameraInfo.srv" NAME_WE)
add_dependencies(px_comm_generate_messages_py _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/CameraInfo.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_py _px_comm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/su/neo_ws/src/px-ros-pkg/px_comm/msg/OpticalFlow.msg" NAME_WE)
add_dependencies(px_comm_generate_messages_py _px_comm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(px_comm_genpy)
add_dependencies(px_comm_genpy px_comm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS px_comm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/px_comm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(px_comm_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(px_comm_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/px_comm
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(px_comm_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(px_comm_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/px_comm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(px_comm_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(px_comm_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/px_comm
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(px_comm_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(px_comm_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/px_comm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(px_comm_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(px_comm_generate_messages_py std_msgs_generate_messages_py)
endif()
