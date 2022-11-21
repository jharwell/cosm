/**
 * \file base_force.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_force
 * \ingroup apf2D
 *
 * \brief Contains common functionality for all virtual forces.
 */
class base_force {
 public:
  /**
   * \brief Modify velocity odometry to pe positive definition.
   *
   * In order to have the some forces work properly, we need to have a
   * velocity with a non-zero length and the correct heading angle at all
   * times. So we report that we have velocity even though we do not, for the
   * purposes of making those calculations work.
   *
   * There probably is a better way to do this, but I don't know what it is.
   */
  static rmath::vector2d make_vel_floor(rmath::vector2d vel);

  base_force(void) = default;
  virtual ~base_force(void) = default;

  /* Not move/copy constructable/assignable by default */
  base_force(const base_force&) = delete;
  base_force& operator=(const base_force&) = delete;
  base_force(base_force&&) = delete;
  base_force& operator=(base_force&&) = delete;



 private:
  /* clang-format off */
  /* clang-format on */
};



} /* namespace cosm::apf2D */
