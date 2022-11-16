/**
 * \file battery_sensor.hpp
 *
 * \copyright 2018 The Nimer Inc., All rights reserved.
 * SPDX-License-Identifier: MIT
 */

#pragma once

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>

#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/sensors/battery_sensor_reading.hpp"
#include "cosm/hal/sensors/metrics/battery_metrics.hpp"

/******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace argos {
class CCI_BatterySensor;
} /* namespace argos */

NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_battery_sensor = std::is_same<TSensor, ::argos::CCI_BatterySensor>;

NS_END(detail);

/******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Battery sensor wrapper for the following supported robots:
 *
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class battery_sensor_impl final : public rer::client<battery_sensor_impl<TSensor>>,
                                  public chsensors::metrics::battery_metrics,
                                  public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;


  explicit battery_sensor_impl(impl_type * const sensor)
      : ER_CLIENT_INIT("cosm.hal.argos.sensors.battery"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {}

  /* move only constructible/assignable for use with saa subsystem */
  const battery_sensor_impl& operator=(const battery_sensor_impl&) = delete;
  battery_sensor_impl(const battery_sensor_impl&) = delete;
  battery_sensor_impl& operator=(battery_sensor_impl&&) = default;
  battery_sensor_impl(battery_sensor_impl&&) = default;

  /* Battery metrics */
  double percent_remaining(void) const override {
    return readings()[0].percentage;
  }

  /**
   * \brief Get the current battery sensor reading for the footbot robot.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_battery_sensor<U>::value)>
  std::vector<chsensors::battery_sensor_reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto temp = decoratee()->GetReading();

    chsensors::battery_sensor_reading ret;
    ret.percentage = temp.AvailableCharge;
    ret.time_left = temp.TimeLeft;

    return {ret};
  }
};

using battery_sensor = battery_sensor_impl<::argos::CCI_BatterySensor>;

NS_END(sensors, argos, hal, cosm);
