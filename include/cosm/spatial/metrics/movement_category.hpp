/**
 * \file movement_category.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief For use in attaching semantic meaning to different categorys of robot
 * motion as part of model validation.
 */
enum movement_category {
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

NS_END(metrics, spatial, cosm);
