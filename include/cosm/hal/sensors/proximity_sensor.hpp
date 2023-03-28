/**
 * \file proximity_sensor.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <limits>
#include <utility>
#include <boost/optional.hpp>

#include "cosm/hal/hal.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/sensors/ir_sensor.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/sensors/lidar_sensor.hpp"
#endif

#include "rcppsw/er/client.hpp"

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"
#include "cosm/hal/sensors/identify.hpp"
#include "cosm/hal/sensors/base_sensor.hpp"
#include "cosm/hal/sensors/stub_sensor.hpp"
#include "cosm/hal/sensors/proximity_sensor_reading.hpp"


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)

#if defined(COSM_HAL_TARGET_HAS_PROX_SENSOR)
using proximity_sensor_impl_type = chargos::sensors::ir_sensor;
#else
using proximity_sensor_impl_type = chsensors::stub_sensor<
  std::vector<chsensors::proximity_sensor_reading>
  >;
#endif

#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using proximity_sensor_impl_type = chros::sensors::lidar_sensor;
#endif

/**
 * \class proximity_sensor
 * \ingroup hal sensors
 *
 * \brief Proxmity sensor wrapper to provide a uniform interface to sensing "is
 * there something near me" regardless of robot and platform. Provides some
 * additional higher level functionality beyond raw sensor readings too.
 */
class proximity_sensor final : public rer::client<proximity_sensor>,
                               public chsensors::base_sensor<proximity_sensor_impl_type> {
 public:
  using impl_type = chsensors::base_sensor<proximity_sensor_impl_type>;
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
  proximity_sensor(proximity_sensor_impl_type&& sensor,
                   const config::proximity_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.sensors.proximity"),
        impl_type(std::move(sensor)),
        m_config(*config) {
    enable();
  }
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
  proximity_sensor(proximity_sensor_impl_type&& sensor,
                   const config::proximity_sensor_config* const config)
      : ER_CLIENT_INIT("cosm.hal.sensors.proximity"),
        impl_type(std::move(sensor)),
        m_config(*config) {
    enable();
  }
#endif

  /* move only constructible/assignable to work with the saa subsystem */
  proximity_sensor(proximity_sensor&&) = default;
  proximity_sensor& operator=(proximity_sensor&&) = default;
  proximity_sensor(const proximity_sensor&) = delete;
  proximity_sensor& operator=(const proximity_sensor&) = delete;

  /* base_sensor overrides */
  void reset(void) override { decoratee().reset(); }
  void enable(void) override { decoratee().enable(); }
  void disable(void) override {decoratee().disable(); }
  bool is_enabled(void) const override { return decoratee().is_enabled(); }

  std::vector<proximity_sensor_reading> readings(void) const {
    return decoratee().readings();
  }

  /**
   * \brief Return the average object reading within proximity for the
   * robot. Proximity is defined as within the "go straight" angle range for the
   * robot
   *
   * If there are not enough objects meeting this criteria that are close enough
   * such that the average distance to them is > than the provided delta,
   * nothing is returned.
   */
  boost::optional<rmath::vector2d> avg_prox_obj(void) const {
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    rmath::vector2d accum;

    size_t count = 0;

    /* constructed, not returned, so we need a copy to iterate over */
    auto readings = this->readings();

    for (auto& r : readings) {
      accum += r;
      ++count;
    }

    if (count == 0) {
      return boost::none;
    }
    ER_TRACE("Processed %zu non-zero readings", count);
    ER_TRACE("Accum: %s@%s",
             rcppsw::to_string(accum).c_str(),
              rcppsw::to_string(accum.angle()).c_str());

    accum /= count;

    ER_DEBUG("Avg obstacle: %s@%s",
             rcppsw::to_string(accum).c_str(),
             rcppsw::to_string(accum.angle()).c_str());

    if (m_config.fov.contains(accum.angle()) &&
        accum.length() <= m_config.delta) {
      return boost::none;
    } else {
      return boost::make_optional(accum);
    }
  }
 private:
  /* clang-format off */
  config::proximity_sensor_config m_config;
  /* clang-format on */
};

} /* namespace cosm::hal::sensors */
