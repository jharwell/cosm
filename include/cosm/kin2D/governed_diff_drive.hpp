/**
 * \file governed_diff_drive.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <utility>

#include "cosm/hal/actuators/diff_drive_actuator.hpp"
#include "cosm/kin2D/diff_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin2D::config {
struct diff_drive_config;
}
namespace cosm::tv {
class switchable_tv_generator;
} /* namespace tv */

namespace cosm::kin2D {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class kin2D::governed_diff_drive
 * \ingroup kin2D
 *
 * \brief A differential drive whose maximum speed can be set in a temporally
 * varyng manner via one or more \ref ctv::switchable_tv_generators, if
 * configured to do so. The effect of all configured generators is cumulative.
 */
class governed_diff_drive final : public kin2D::diff_drive {
 public:
  governed_diff_drive(const config::diff_drive_config* config,
                      hal::actuators::diff_drive_actuator&& actuator)
      : diff_drive(config, std::move(actuator)) {}

  /* move only constructible/assignable to work with the saa subsystem */
  governed_diff_drive(governed_diff_drive&&) = default;
  governed_diff_drive& operator=(governed_diff_drive&&) = default;
  governed_diff_drive(const governed_diff_drive&) = delete;
  governed_diff_drive& operator=(const governed_diff_drive&) = delete;

  /**
   * \brief Get the current value of the governor.
   *
   * \return A percent [0.0,1.0].
   */
  double active_throttle(void) const RCPPSW_PURE;

  /**
   * \brief Get the current value of the governor if it was active (it might not
   * be).
   *
   * \return A percent [0.0,1.0].
   */
  double applied_throttle(void) const RCPPSW_PURE;

  /**
   * \brief Add a variance generate to the list of candidate generators. This is
   * a function, rather than part of the constructor because the creation of
   * variance generators is only performed if a certain type of variance is
   * enabled (e.g. motion throttling when certain conditions are met).
   */
  void tv_generator(const tv::switchable_tv_generator* generator) {
    m_generators.push_back(generator);
  }

 private:
  /* clang-format off */
  std::vector<const tv::switchable_tv_generator*> m_generators{};
  /* clang-format on */
};

} /* namespace cosm::kin2D */
