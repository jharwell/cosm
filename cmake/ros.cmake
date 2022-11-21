#
# Copyright 2022 John Harwell, All rights reserved.
#
# SPDX-License-Identifier: MIT
#
macro(cosm_ros_configure_components)
  string(CONCAT ros_regex
    "src/ros|"
    "src/hal/ros|"
    "src/pal/ros|"
    "src/fsm|"
    "src/foraging/fsm|"
    "src/kin2D|"
    "src/foraging/metrics|"
    "src/spatial/fsm|"
    "src/spatial/metrics|"
    "src/spatial/strategy|"
    "src/repr/operations/block_pickup|"
    "src/repr/base_block3D|"
    "src/repr/config/xml"
    )

  if(${COSM_HAL_TARGET_OPERATES_IN_Q3D})
    string(CONCAT ros_regex "${ros_regex} | ${Q3D_bindings_regex}")
  endif()

  if(${COSM_HAL_TARGET_OPERATES_IN_3D})
    string(CONCAT ros_regex "${ros_regex} | ${3D_bindings_regex}")
  endif()

  libra_component_register_as_src(
    cosm_ros_SRC
    cosm
    "${cosm_SRC}"
    ros
    "(${ros_regex})")

  # Root project (not used in find_package())
  if (NOT cosm_FIND_COMPONENTS)
    set(cosm_FIND_COMPONENTS
      common
      ros
      )
  endif()

endmacro()
