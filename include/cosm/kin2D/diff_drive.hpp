/**
 * @file diff_drive.hpp
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

#ifndef INCLUDE_COSM_KIN2D_DIFF_DRIVE_HPP_
#define INCLUDE_COSM_KIN2D_DIFF_DRIVE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"
#include "cosm/hal/actuators/diff_drive_actuator.hpp"
#include "cosm/kin2D/diff_drive_fsm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);
namespace config {
struct diff_drive_config;
}

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/

/**
 * @class diff_drive
 * @ingroup cosm kin2D
 *
 * @brief Model for a TWO wheeled diff drive robot, providing a number of
 * operating modes:
 *
 * \ref drive_type::kTankDrive
 * \ref drive_type::kFSMDrive
 */
class diff_drive : public rer::client<diff_drive> {
 public:
  enum class drive_type {
    kTankDrive, /// Controls like those of a tank
    kFSMDrive   /// Control via soft/hard turn FSM
  };

  /**
   * @brief Initialize diff drive kin model.
   *
   * @param type The drive type; see \ref drive_type
   * @param actuator The underlying differential steering actuator (via HAL)
   * @param config Configuration.
   */
  diff_drive(const config::diff_drive_config* config,
             const hal::actuators::diff_drive_actuator& actuator,
             drive_type type);

  const diff_drive& operator=(const diff_drive&) = delete;
  diff_drive(const diff_drive&) = default;

  void reset(void) { m_actuator.reset(); }
  double max_speed(void) const { return m_max_speed; }

  /*
   * @brief Gets a new speed/heading angle and transforms it into wheel commands
   * via an FSM.
   *
   * @param desired_speed The new linear speed of the robot. Can be any value,
   *        but is clamped by internal FSM to the maximum speed set during
   *        construction.
   * @param angle_delta The difference from the robot's CURRENT heading
   *                    (i.e."change this much from the direction you are
   *                    currently going in").
   *
   * @return \ref status_t.
   */
  status_t fsm_drive(double desired_speed, const rmath::radians& angle_delta);

  /**
   * Tank drive method (i.e. how a tank moves.)
   *
   * @param left_speed The robot left side's speed along the X axis
   *                   [-1.0..1.0]. Forward is positive.
   * @param right_speed The robot right side's speed along the X axis
   *                    [-1.0..1.0]. Forward is positive.
   * @param square_inputs If set, decreases the input sensitivity at low
   *                      speeds.
   * @return \ref status_t.
   */
  status_t tank_drive(double left_speed, double right_speed, bool square_inputs);

 private:
  /**
   * @brief Limit the value to [-1, 1].
   */
  double limit(double value) const RCSW_PURE;

  /* clang-format off */
  bool                                m_hard_turn{false};
  drive_type                          m_drive_type;
  double                              m_max_speed;
  diff_drive_fsm                      m_fsm;
  hal::actuators::diff_drive_actuator m_actuator;
  /* clang-format on */
};

NS_END(kin2D, cosm);

#endif /* INCLUDE_COSM_KIN2D_DIFF_DRIVE_HPP_ */
