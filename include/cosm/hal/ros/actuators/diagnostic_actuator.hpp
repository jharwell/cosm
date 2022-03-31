 /**
 * \file diagnostic_actuator.hpp
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
#include <map>

#include "rcppsw/utils/color.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/actuators/diagnostics.hpp"
#include "cosm/hal/ros/actuators/ros_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class diagnostic_actuator
 * \ingroup hal ros actuators
 *
 * \brief Diagnostic actuator.
 *
 *  Supports the following robots:
 *
 * - ROS turtlebot3 (stub until this is implemented in hardware).
 */
class diagnostic_actuator final : public rer::client<diagnostic_actuator>,
                                  public chros::actuators::ros_actuator {
 public:
  using map_type = std::map<uint8_t, rutils::color>;

  explicit diagnostic_actuator(bool enable,
                               const map_type& map)
      : ER_CLIENT_INIT("cosm.hal.ros.actuators.diagnostic"),
        ros_actuator(cros::topic()),
        m_enabled(enable),
        m_map(map) {}

  /* copy constructible/assignable to work with the saa subsystem */
  diagnostic_actuator(const diagnostic_actuator&) = delete;
  diagnostic_actuator& operator=(const diagnostic_actuator&)= delete;
  diagnostic_actuator(diagnostic_actuator&&) = default;
  diagnostic_actuator& operator=(diagnostic_actuator&&)= default;

  void reset(void) override { disable(); }
  void enable(void) override { m_enabled = true; }
  bool is_enabled(void) const override { return m_enabled; }
  void disable(void) override { m_enabled = false; }

  void emit(uint8_t type) {
    if (!is_enabled()) {
      return;
    }
    auto it = m_map.find(type);
    ER_ASSERT(it != m_map.end(),
              "Unknown diagnostic category %d",
              type);
  }

  /* clang-format off */
  bool     m_enabled;
  map_type m_map;
  /* clang-format on */
};

NS_END(actuators, ros, hal, cosm);
