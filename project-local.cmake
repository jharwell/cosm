################################################################################
# Configuration Options                                                        #
################################################################################
set(WITH_HAL_CONFIG "argos-footbot" CACHE STRING "Specify the Hardware Abstraction Layer (HAL) target")
define_property(
  CACHED_VARIABLE PROPERTY "WITH_HAL_CONFIG"
		BRIEF_DOCS "Specify the Hardware Abstraction Layer (HAL) target"
		FULL_DOCS "Must be exactly one of: [argos-footbot]"
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
if (BUILD_FOR_MSI)
  set(LOCAL_INSTALL_PREFIX /home/gini/shared/swarm/$ENV{MSICLUSTER})
elseif(BUILD_FOR_TRAVIS)
  set(LOCAL_INSTALL_PREFIX /usr/local)
else()
  set(LOCAL_INSTALL_PREFIX /opt/data/local)
endif()

set(${target}_INCLUDE_DIRS
  "${${target}_INC_PATH}"
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

if (BUILD_FOR_MSI)
  set(${target}_LIBRARY_DIRS ${${target}_LIBRARY_DIRS} ${LOCAL_INSTALL_PREFIX}/lib/argos3)
endif()

if (NOT TARGET ${target})
  add_library(${target} STATIC ${${target}_SRC})
  target_link_libraries(${target} ${${target}_LIBRARIES})
  target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
  target_include_directories(${target} SYSTEM PRIVATE "${${target}_SYS_INCLUDE_DIRS}")

  if ("${WITH_HAL_CONFIG}" MATCHES "argos-footbot")
    target_include_directories(${target} SYSTEM PRIVATE /usr/include/lua5.2)
    target_compile_definitions(${target} PUBLIC HAL_CONFIG=HAL_CONFIG_ARGOS_FOOTBOT)
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
