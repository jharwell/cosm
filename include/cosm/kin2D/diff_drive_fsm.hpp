/**
 * \file diff_drive_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <utility>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/simple_fsm.hpp"
#include "cosm/kin2D/config/diff_drive_config.hpp"

#include "cosm/cosm.hpp"
#include "cosm/kin/twist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diff_drive_fsm
 * \ingroup kin2D
 *
 * \brief Handles the control of the diff drive for the robot, in terms
 * of how quickly/how much to change wheel speeds. Does NOT actually change the
 * wheel speeds.
 */
class diff_drive_fsm final : public rpfsm::simple_fsm {
 public:
  diff_drive_fsm(const ckin2D::config::diff_drive_config* config);

  /* move only constructible/assignable to work with the saa subsystem */
  diff_drive_fsm(diff_drive_fsm&&);
  diff_drive_fsm& operator=(diff_drive_fsm&&);

  /*
   * \brief Gets a direction vector as input and transforms it into wheel
   * speeds internally.
   *
   * \param delta_vel The difference from the robot's CURRENT heading
   *                  (i.e."change this much from the direction you are
   *                  currently going in") is computed according to \p old_vel.
   */
  void change_velocity(const ckin::twist& delta);

  const ckin::twist& configured_twist(void) const { return m_twist; }

 private:
  /**
   * \brief Output new wheel speeds according to the desired change in velocity.
   *
   * \param speed1 Speed of left wheel.
   *
   * \param speed2 Speed of right wheel.
   *
   * \param new_heading Robot heading, which is used to determine which speed to
   *                    apply to which wheel, so that the proper turn direction
   *                    is executed.
   */
  void configure_wheel_speeds(double speed1,
                              double speed2,
                              const rmath::radians& new_heading);
  /**
   * \brief Output new twist according to the desired change in velocity.
   *
   * \param speed Desired speed.
   *
   * \param heading Desired robot heading.
   */
  void configure_twist(double speed, const rmath::radians& heading);

  /*
   * @enum The robot can be in three different turning states.
   */
  enum fsm_states {
    /**
     * Both wheels rotating forward at slightly different speeds
     */
    ekST_SOFT_TURN,
    /**
     * Wheels are turning with opposite & max speeds
     */
    ekST_HARD_TURN,
    ekST_MAX_STATES
  };

  /**
   * \brief Turning data for input into the state machine, to translate the
   * desired heading change into wheel speeds.
   */
  struct turn_data final : public rpfsm::event_data {
    turn_data(double speed_in, const rmath::radians& angle_in)
        : speed(speed_in), angle(angle_in) {}

    double speed;
    rmath::radians angle;
  };

  /**
   * \brief Robots in this state will execute a gradual turn in the desired
   * heading direction. Threshold for this type of turn is controlled by
   * parameters.
   */
  RCPPSW_FSM_STATE_DECLARE(diff_drive_fsm, soft_turn, turn_data);

  /**
   * \brief Robots in this state will execute an in-place turn (a spin really)
   * in the direction of the desired heading. Threshold for this type of turn
   * is controlled by parameters.
   */
  RCPPSW_FSM_STATE_DECLARE(diff_drive_fsm, hard_turn, turn_data);

  RCPPSW_FSM_DEFINE_STATE_MAP_ACCESSOR(state_map, index) override {
    return &mc_state_map[index];
  }
  /* clang-format off */
  const config::diff_drive_config mc_config;

  ckin::twist                     m_twist{};

  RCPPSW_FSM_DECLARE_STATE_MAP(state_map,
                        mc_state_map,
                        ekST_MAX_STATES);
  /* clang-format on */
};

NS_END(kin2D, cosm);
