/**
 * \file argos_actuator.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/hal/actuators/base_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class argos_actuator
 * \ingroup hal argos actuators
 *
 * \brief Base actuator class to provide a common interface to all ARGoS
 *        actuators.
 */
template <typename TActuator>
class argos_actuator : public rer::client<argos_actuator<TActuator>>,
                       public chactuators::base_actuator<TActuator*> {
 public:
  using impl_type = TActuator*;
  using chactuators::base_actuator<impl_type>::decoratee;

  explicit argos_actuator(TActuator* actuator)
      : ER_CLIENT_INIT("cosm.hal.actuator.argos_actuator"),
        chactuators::base_actuator<TActuator*>(std::move(actuator)) {}

  virtual ~argos_actuator(void) = default;

  argos_actuator(const argos_actuator&) = delete;
  argos_actuator& operator=(const argos_actuator&) = delete;
  argos_actuator(argos_actuator&&) = default;
  argos_actuator& operator=(argos_actuator&&) = default;

  void enable(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    m_enabled = true;
  }

  void disable(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    reset();
    m_enabled = false;
  }

  void reset(void) override {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Reset();
  }

  bool is_enabled(void) const override { return m_enabled; }

 private:
  /* clang-format off */
  bool m_enabled{true};
  /* clang-format on */
};

NS_END(actuators, argos, hal, cosm);

