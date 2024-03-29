/**
 * \file light_sensor.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_light_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/sensors/stub_sensor.hpp"
#include "cosm/hal/sensors/light_sensor_reading.hpp"
#include "cosm/hal/argos/sensors/detail/identify.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class light_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Light sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot^
 * - ARGoS epuck^
 *
 * ^The simulated sensor is expensive to update each timestep, AND is not
 *  necessarily needed each timestep, so it is disabled upon creation, so robots
 *  can selectively enable/disable it as needed for maximum computational
 *  efficiency.
 *
 * For other HAL targets, the sensor is stubbed out so things will compile
 * elsewhere, but the sensor and any algorithms using it WILL NOT WORK.
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class light_sensor_impl final : public rer::client<light_sensor_impl<TSensor>>,
                                public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  using reading_type = chsensors::light_sensor_reading;

  explicit light_sensor_impl(impl_type * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.light"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {
    disable();
  }

  /* move only constructible/assignable for use with saa subsystem */
  const light_sensor_impl& operator=(const light_sensor_impl&) = delete;
  light_sensor_impl(const light_sensor_impl&) = delete;
  light_sensor_impl& operator=(light_sensor_impl&&) = default;
  light_sensor_impl(light_sensor_impl&&) = default;

  /**
   * \brief Get the current light sensor readings for the footbot/epuck robots.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_light_sensor<U>::value ||
                                  detail::is_epuck_light_sensor<U>::value)>
  std::vector<reading_type> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading_type> ret;
    for (auto &r : decoratee()->GetReadings()) {
      ret.push_back({r.Value, rmath::radians(r.Angle.GetValue())});
    } /* for(&r..) */

    return ret;
  }

  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(chsensors::is_null_sensor<U>::value)>
  std::vector<reading_type>  readings(void) const {
    return {};
  }
};

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using light_sensor = light_sensor_impl<::argos::CCI_FootBotLightSensor>;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using light_sensor = light_sensor_impl<::argos::CCI_EPuckLightSensor>;
#else
using light_sensor = chsensors::stub_sensor<
  std::vector<chsensors::light_sensor_reading>
  >;
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal::argos::sensors */
