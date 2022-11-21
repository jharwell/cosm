/**
 * \file argos_actuator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::hal::argos::actuators {

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

  ~argos_actuator(void) override = default;

  argos_actuator(const argos_actuator&) = delete;
  argos_actuator& operator=(const argos_actuator&) = delete;
  argos_actuator(argos_actuator&&) = default;
  argos_actuator& operator=(argos_actuator&&) = default;

  void enable(void) override {
    ER_CHECK(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    m_enabled = true;

 error:
    return;
  }

  void disable(void) override {
    ER_CHECK(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    reset();
    m_enabled = false;

 error:
    return;
  }

  void reset(void) override {
    ER_CHECK(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    decoratee()->Reset();

 error:
    return;
  }

  bool is_enabled(void) const override { return m_enabled; }

 private:
  /* clang-format off */
  bool m_enabled{true};
  /* clang-format on */
};

} /* namespace cosm::hal::argos::actuators */
