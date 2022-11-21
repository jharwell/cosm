/**
 * \file swarm_visualizer_adaptor.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/pal/base_swarm_visualizer.hpp"

#include "cosm/hal/robot.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::argos {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_visualizer_adaptor
 * \ingroup pal argos
 *
 \brief Adaptor for \ref base_swarm_visualizer to provide an interface for
 * visualizing swarms within ARGoS.
 */
class swarm_visualizer_adaptor : public cpal::base_swarm_visualizer,
                                 public ::argos::CQTOpenGLUserFunctions {
 public:
  swarm_visualizer_adaptor(void) = default;

  /* Not move/copy constructable/assignable by default */
  swarm_visualizer_adaptor(const swarm_visualizer_adaptor&) = delete;
  swarm_visualizer_adaptor& operator=(const swarm_visualizer_adaptor&) = delete;
  swarm_visualizer_adaptor(swarm_visualizer_adaptor&&) = delete;
  swarm_visualizer_adaptor& operator=(swarm_visualizer_adaptor&&) = delete;

 private:
  /* clang-format off */
  /* clang-format on */
};

} /* namespace cosm::pal::argos */
