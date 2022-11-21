/**
 * \file contexts.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/context.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
enum context_type {
  /**
   * \brief Robots which are returning to a nest via the homing action.
   */
  ekHOMING,

  /**
   * \brief Robots which are exploring.
   */
  ekEXPLORING,

  /**
   * \brief Robots which are flocking.
   */
  ekFLOCKING,

  /**
   * \brief Catch-all category which is reported each timestep.
   */
  ekALL,
  ekMAX
};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
/**
 * \brief For use in attaching semantic meaning to different categories of robot
 * motion as part of model validation.
 */
extern std::vector<rmetrics::context> kContexts;

} /* namespace cosm::kin::metrics */
