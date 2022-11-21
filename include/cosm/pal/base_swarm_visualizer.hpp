/**
 * \file base_swarm_visualizer.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/hal/robot.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_swarm_visualizer
 * \ingroup pal
 *
 * \brief Base class for visualizing live swarm behaviors as it runs.
 *
 * Only core functionality agnostic to the platform on which the swarm control
 * algorithms are being executed is included here.
 */
class base_swarm_visualizer {
 public:
  base_swarm_visualizer(void) = default;
  virtual ~base_swarm_visualizer(void) = default;

  /* Not copy constructable/assignable by default */
  base_swarm_visualizer(const base_swarm_visualizer&) = delete;
  const base_swarm_visualizer& operator=(const base_swarm_visualizer&) = delete;

 private:
  /* clang-format off */
  /* clang-format on */
};

} /* namespace cosm::pal */
