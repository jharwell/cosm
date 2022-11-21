#
# Copyright 2022 John Harwell, All rights reserved.
#
# SPDX-License-Identifier: MIT
#
macro(cosm_argos_configure_components)
  string(CONCAT argos_regex
    "src/ds|"
    "src/arena|"
    "src/convergence|"
    "src/fsm|"
    "src/hal/argos/subsystem|"
    "src/hal/argos/sensors|"
    "src/metrics|"
    "src/oracle|"
    "src/pal/argos|"
    "src/repr|"
    "src/spatial/common|"
    "src/spatial/fsm|"
    "src/foraging|"
    "src/spatial/metrics|"

    # While these are really for 2D agents, they compile OK for flying
    # agents and could theoretically be used in that context someday.
    "src/spatial/strategy/base|"
    "src/spatial/strategy/nest|"
    "src/spatial/strategy/explore|"

    "src/tv|"
    "src/argos"
    )

  if(${COSM_HAL_TARGET_OPERATES_IN_Q3D})
    # Foraging is a ground robot only thing in ARGoS.
    string(CONCAT argos_regex
      "${argos_regex}|"
      "${Q3D_bindings_regex}|"
      "src/spatial/strategy/blocks")
  endif()

  if(${COSM_HAL_TARGET_OPERATES_IN_3D})
    string(CONCAT argos_regex
      "${argos_regex}|"
      "${3D_bindings_regex}")
  endif()

  libra_component_register_as_src(
    cosm_argos_SRC
    cosm
    "${cosm_SRC}"
    argos
    "${argos_regex}")

  if(COSM_WITH_VIS)
    libra_component_register_as_src(
      cosm_argos_vis_SRC
      cosm
      "${cosm_SRC}"
      argos_vis
      "src/argos/vis")
  endif()

  # Root project (not used in find_package())
  if (NOT cosm_FIND_COMPONENTS)
    set(cosm_FIND_COMPONENTS
      common
      argos
      argos_vis
      )
  endif()
endmacro()
