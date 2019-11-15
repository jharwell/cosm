################################################################################
# Configuration Options                                                        #
################################################################################
set(COSM_HAL_TARGET "None" CACHE STRING "Specify the Hardware Abstraction Layer (HAL) target")
define_property(
  CACHED_VARIABLE PROPERTY "COSM_HAL_TARGET"
		BRIEF_DOCS "Specify the Hardware Abstraction Layer (HAL) target"
		FULL_DOCS "Must be exactly one of: [argos-footbot,ev3]"
	        )
set(${target}_CHECK_LANGUAGE "CXX")

################################################################################
# External Projects                                                            #
################################################################################
# RCPPSW
add_subdirectory(ext/rcppsw)

################################################################################
# Includes                                                                     #
################################################################################
if("${LIBRA_BUILD_FOR}" MATCHES "MSI" )
  message(STATUS "Building for MSI")
  set(LOCAL_INSTALL_PREFIX /home/gini/shared/swarm/$ENV{MSICLUSTER})
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${LIBRA_BUILD_FOR}" MATCHES "TRAVIS")
  message(STATUS "Building for TRAVIS")
  set(LOCAL_INSTALL_PREFIX /usr/local)
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${LIBRA_BUILD_FOR}" MATCHES "ARGOS")
  message(STATUS "Building for ARGoS")
  set(LOCAL_INSTALL_PREFIX /opt/data/local)
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${LIBRA_BUILD_FOR}" MATCHES "EV3")
  message(STATUS "Building for EV3")
    set(COSM_HAL_TARGET "ev3")
else()
  message(FATAL_ERROR
    "Unknown build target '${LIBRA_BUILD_FOR}'. Must be: [MSI,TRAVIS,ARGOS,EV3]")
endif()

set(${target}_INCLUDE_DIRS
  ${${target}_INC_PATH}
  ${rcppsw_INCLUDE_DIRS})
set(${target}_SYS_INCLUDE_DIRS
  ${rcppsw_SYS_INCLUDE_DIRS}
  ${LOCAL_INSTALL_PREFIX}/include)

################################################################################
# Libraries                                                                    #
################################################################################
set(${target}_LIBRARIES
  rcppsw
  )

set(${target}_LIBRARY_DIRS ${rcppsw_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

if ("${LIBRA_BUILD_FOR}" MATCHES "MSI")
  set(${target}_LIBRARY_DIRS
    ${${target}_LIBRARY_DIRS}
    ${LOCAL_INSTALL_PREFIX}/lib/argos3)
endif()

if (NOT TARGET ${target})
  add_library(${target} STATIC ${${target}_SRC})
  target_link_libraries(${target} ${${target}_LIBRARIES})
  target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
  target_include_directories(${target} SYSTEM PRIVATE "${${target}_SYS_INCLUDE_DIRS}")

  if ("${COSM_HAL_TARGET}" MATCHES "argos-footbot")
    target_include_directories(${target} SYSTEM PRIVATE /usr/include/lua5.2)
    target_compile_definitions(${target} PUBLIC COSM_HAL_TARGET=HAL_TARGET_ARGOS_FOOTBOT)
  elseif("${COSM_HAL_TARGET}" MATCHES "ev3")
    target_compile_definitions(${target} PUBLIC COSM_HAL_TARGET=HAL_TARGET_EV3)
  else()
    message(FATAL_ERROR "Bad HAL Target ${COSM_HAL_TARGET}. Must be [ev3,argos-footbot]")
  endif()
endif()


################################################################################
# Exports                                                                      #
################################################################################
if (NOT IS_ROOT_PROJECT)
  set(${target}_INCLUDE_DIRS "${${target}_INCLUDE_DIRS}" PARENT_SCOPE)
  set(${target}_SYS_INCLUDE_DIRS "${${target}_SYS_INCLUDE_DIRS}" PARENT_SCOPE)
  set(${target}_LIBRARIES "${${target}_LIBRARIES}" PARENT_SCOPE)
  set(${target}_LIBRARY_DIRS "${${target}_LIBRARY_DIRS}" PARENT_SCOPE)
endif()
