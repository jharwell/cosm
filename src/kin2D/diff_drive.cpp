/**
 * \file diff_drive.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin2D/diff_drive.hpp"

#include "cosm/kin2D/config/diff_drive_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
diff_drive::diff_drive(const config::diff_drive_config* const config,
                       chactuators::diff_drive_actuator&& actuator)
    : ER_CLIENT_INIT("cosm.kin2D.diff_drive"),
      m_config(*config),
      m_fsm(config),
      m_actuator(std::move(actuator)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive::fsm_drive(const ckin::twist& delta) {
  m_fsm.change_velocity(delta);

  /* don't need to normalize--done by fsm internally */
  rmath::range<rmath::radians> range(-m_config.soft_turn_max,
                                     m_config.soft_turn_max);

  m_actuator.set_from_twist(m_fsm.configured_twist(),
                            range,
                            m_config.max_linear_speed);
} /* fsm_drive() */

NS_END(kin2D, cosm);
