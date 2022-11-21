/**
 * \file ir_sensor.hpp
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
#include <limits>
#include <boost/optional.hpp>

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_proximity_sensor.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#endif /* COSM_HAL_TARGET */

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/radians.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/sensors/stub_sensor.hpp"
#include "cosm/hal/argos/sensors/detail/identify.hpp"
#include "cosm/hal/sensors/proximity_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ir_sensor_impl
 * \ingroup hal argos sensors
 *
 * \brief Proxmity sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS foot-bot
 * - ARGoS e-puck
 *
 * For other HAL targets, the sensor is stubbed out so things will compile
 * elsewhere, but the sensor and any algorithms using it WILL NOT WORK.
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class ir_sensor_impl : public rer::client<ir_sensor_impl<TSensor>>,
                       public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  explicit ir_sensor_impl(TSensor * const sensor)
       : ER_CLIENT_INIT("cosm.hal.argos.sensors.ir"),
         chargos::sensors::argos_sensor<TSensor>(sensor) {}

  /* move only constructible/assignable for use with saa subsystem */
  const ir_sensor_impl& operator=(const ir_sensor_impl&) = delete;
  ir_sensor_impl(const ir_sensor_impl&) = delete;
  ir_sensor_impl& operator=(ir_sensor_impl&&) = default;
  ir_sensor_impl(ir_sensor_impl&&) = default;

  /**
   * \brief Get the current ir sensor readings for the footbot robot.
   *
   * \return A vector of (X,Y) pairs of sensor readings corresponding to
   * object distances.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_footbot_ir_sensor<U>::value ||
                                  detail::is_epuck_ir_sensor<U>::value)>
  std::vector<chsensors::proximity_sensor_reading> readings(void) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);

    std::vector<rmath::vector2d> ret;
    for (auto &r : decoratee()->GetReadings()) {
      /*
       * Value of 0 means nothing in range, don't include it for further
       * processing--small optimization.
       */
      if (r.Value > 0) {
        ret.emplace_back(r.Value, rmath::radians(r.Angle.GetValue()));
      }
    } /* for(&r..) */

    return ret;
  }

  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(chsensors::is_null_sensor<U>::value)>
  std::vector<chsensors::proximity_sensor_reading>  readings(void) const {
    return {};
  }
};

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
using ir_sensor = ir_sensor_impl<::argos::CCI_FootBotProximitySensor>;
#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
using ir_sensor = ir_sensor_impl<::argos::CCI_EPuckProximitySensor>;
#else
using ir_sensor = chsensors::stub_sensor<
                  std::vector<chsensors::proximity_sensor_reading>
                  >;
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal::argos::sensors */
