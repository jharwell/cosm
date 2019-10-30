/**
 * @file governed_diff_drive.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_KIN2D_GOVERNED_DIFF_DRIVE_HPP_
#define INCLUDE_COSM_KIN2D_GOVERNED_DIFF_DRIVE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/actuators/diff_drive_actuator.hpp"
#include "cosm/kin2D/diff_drive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);
namespace kin2D::config {
struct diff_drive_config;
}
namespace tv {
class switchable_tv_generator;
} /* namespace tv */

NS_START(kin2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class governed_diff_drive
 * @ingroup cosm kin2D
 *
 * @brief A differential drive whose maximum speed can be set in a temporally
 * varyng manner via \ref tv::switchable_tv_generator, if configured to do so.
 */
class governed_diff_drive final : public kin2D::diff_drive {
 public:
  governed_diff_drive(const config::diff_drive_config* config,
                      const hal::actuators::diff_drive_actuator& actuator,
                      drive_type type)
      : diff_drive(config, actuator, type) {}

  const governed_diff_drive& operator=(const governed_diff_drive&) = delete;
  governed_diff_drive(const governed_diff_drive&) = default;

  /**
   * @brief Get the current value of the governor.
   *
   * @return A percent [0.0,1.0]
   */
  double active_throttle(void) const RCSW_PURE;

  /**
   * @brief Get the current value of the governor if it was active (it might not
   * be).
   *
   * @return A percent [0.0,1.0]
   */
  double applied_throttle(void) const RCSW_PURE;

  /**
   * @brief Set the variance generator. This is a function, rather than part of
   * the constructor because the creation of variance generators is only
   * performed if a certain type of variance is enabled (e.g. motion throttling
   * when certain conditions are met).
   */
  void tv_generator(const tv::switchable_tv_generator* generator) {
    mc_generator = generator;
  }

 private:
  /* clang-format off */
  const tv::switchable_tv_generator* mc_generator{nullptr};
  /* clang-format on */
};

NS_END(kin2D, cosm);

#endif /* INCLUDE_COSM_KIN2D_GOVERNED_DIFF_DRIVE_HPP_ */
