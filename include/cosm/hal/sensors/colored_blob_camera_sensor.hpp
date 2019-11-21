/**
 * \file colored_blob_camera_sensor.hpp
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

#ifndef INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_
#define INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include "rcppsw/rcppsw.hpp"
#include "cosm/hal/hal.hpp"
#include "rcppsw/utils/color.hpp"
#include "rcppsw/math/vector2.hpp"
#include "cosm/cosm.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#else
#error "Selected hardware has no blob camera sensor!"
#endif /* HAL_TARGET */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_argos_blob_camera_sensor = std::is_same<
  TSensor,
  argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class colored_blob_camera_sensor_impl
 * \ingroup hal sensors
 *
 * \brief Omnidirectional colored blob camera sensor wrapper.
 *
 * Supports the following robots:
 *
 * - ARGoS footbot. The simulated sensor is expensive to update each timestep,
 *   so it is disabled upon creation, so robots can selectively enable/disable
 *   it as needed for maximum speed.
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class colored_blob_camera_sensor_impl {
 public:
  /**
   * \brief A camera sensor reading (color, distance, angle) tuple.
   */
  struct reading {
    rmath::vector2d vec;
    rutils::color color;
  };

  explicit colored_blob_camera_sensor_impl(TSensor * const sensor)
      : m_sensor(sensor) {}

  /**
   * \brief Get the sensor readings for the footbot robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_blob_camera_sensor<U>::value)>
  std::vector<reading>  readings(void) const {
    std::vector<reading> ret;
    for (auto &r : m_sensor->GetReadings().BlobList) {
      struct reading s = {
        .vec = {r->Distance, rmath::radians(r->Angle.GetValue())},
        .color = rutils::color(r->Color.GetRed(),
                              r->Color.GetGreen(),
                              r->Color.GetBlue())
      };
      ret.push_back(s);
    } /* for(&r..) */

    return ret;
  }

  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_blob_camera_sensor<U>::value)>
  void enable(void) const { m_sensor->Enable(); }

  template <typename U = TSensor,
            RCPPSW_SFINAE_FUNC(detail::is_argos_blob_camera_sensor<U>::value)>
  void disable(void) const { m_sensor->Disable(); }

 private:
  TSensor* const m_sensor;
};

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
using colored_blob_camera_sensor =
    colored_blob_camera_sensor_impl<argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;
#endif /* HAL_TARGET */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_ */
