/**
 * \file base_actuator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_HAL_ACTUATORS_BASE_ACTUATOR_HPP_
#define INCLUDE_COSM_HAL_ACTUATORS_BASE_ACTUATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_actuator
 * \ingroup hal actuators
 *
 * \brief Base actuator class to provide a common interface to all actuators.
 */
class base_actuator : public rer::client<base_actuator> {
 public:
  base_actuator(void) : ER_CLIENT_INIT("cosm.hal.actuators.base_actuator") {}
  virtual ~base_actuator(void) = default;

  base_actuator(const base_actuator&) = default;
  base_actuator& operator=(const base_actuator&) = default;
  base_actuator(base_actuator&&) = default;
  base_actuator& operator=(base_actuator&&) = default;

  /**
   * \brief Reset the actuator to its initialized state.
   */
  virtual void reset(void) = 0;

  /**
   * \brief Disable the actuator. Future commands to the actuator should either
   * throw an error or do nothing (application dependent) until the actuator is
   * re-enabled.Should do nothing if actuator is already disabled.
   */
  virtual void disable(void) = 0;

  /**
   * \brief Enable the actuator. Should do nothing if actuator is already
   * enabled.
   */
  virtual void enable(void) = 0;
};

NS_END(actuators, hal, cosm);

#endif /* INCLUDE_COSM_HAL_ACTUATORS_BASE_ACTUATOR_HPP_ */
