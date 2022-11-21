 /**
 * \file diagnostic_actuator.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::hal::ros::actuators {

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

} /* namespace cosm::hal::ros::actuators */
