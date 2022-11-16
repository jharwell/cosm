/**
 * \file temporal_penalty.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {
class base_controller;
} /* namespace cosm::controller */

NS_START(cosm, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class temporal_penalty
 * \ingroup tv
 *
 * \brief Handles subjecting a robot to a penalty when doing something via a
 * timeout in which the robot will sit still.
 */
class temporal_penalty {
 public:
  /**
   * \brief Initialize the penalty.
   *
   * \param controller The controller for the robot being subjected to the
   *                   penalty.
   * \param id The ID to associate with the penalty.
   * \param penalty The # of timesteps for the penalty.
   * \param start_time The timestep the penalty will start on.
   */
  temporal_penalty(const controller::base_controller* const controller,
                   const rtypes::type_uuid& id,
                   const rtypes::timestep& penalty,
                   const rtypes::timestep& start_time)
      : mc_id(id),
        mc_penalty(penalty),
        mc_start_time(start_time),
        mc_controller(controller) {}

  temporal_penalty(const temporal_penalty&) = default;
  temporal_penalty(temporal_penalty&&) = default;

  /* Not copy/move assignable by default */
  const temporal_penalty& operator=(const temporal_penalty&) = delete;
  temporal_penalty& operator=(temporal_penalty&&) = delete;

  const controller::base_controller* controller(void) const {
    return mc_controller;
  }
  const rtypes::timestep& start_time(void) const { return mc_start_time; }
  const rtypes::timestep& penalty(void) const { return mc_penalty; }
  const rtypes::type_uuid& id(void) const { return mc_id; }

  bool operator==(const temporal_penalty& other) {
    return this->controller() == other.controller();
  }

  /**
   * \brief If \c TRUE, then the robot has satisfied the block_manipulation
   * penalty.
   */
  bool penalty_satisfied(const rtypes::timestep& current_time) const {
    return current_time - mc_start_time >= mc_penalty;
  }

 private:
  /* clang-format off */
  const rtypes::type_uuid            mc_id;
  const rtypes::timestep             mc_penalty;
  const rtypes::timestep             mc_start_time;
  const controller::base_controller* mc_controller;
  /* clang-format on */
};

NS_END(tv, cosm);
