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

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/hal/hal.hpp"
#include "rcppsw/utils/color.hpp"
#include "cosm/cosm.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

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
 * - ARGoS footbot
 * - ARGoS epuck
 * - ARGoS pipuck
 *
 * ^The simulated sensor is expensive to update each timestep,
 *  so it is disabled upon creation, so robots can selectively enable/disable
 *  it as needed for maximum speed.
 *
 * \tparam TSensor The underlying sensor handle type abstracted away by the
 *                  HAL. If nullptr, then that effectively disables the sensor
 *                  at compile time, and SFINAE ensures no member functions can
 *                  be called.
 */
template <typename TSensor>
class colored_blob_camera_sensor_impl final : public rer::client<colored_blob_camera_sensor_impl<TSensor>> {
 public:
  using impl_type = TSensor;

  /**
   * \brief A camera sensor reading (color, distance in meters) tuple.
   */
  struct reading {
    rmath::vector2d vec{};
    rutils::color color{};
  };

  explicit colored_blob_camera_sensor_impl(TSensor * const sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.colored_blob_camera"),
        m_sensor(sensor) {
    disable();
  }

  const colored_blob_camera_sensor_impl& operator=(const colored_blob_camera_sensor_impl&) = delete;
  colored_blob_camera_sensor_impl(const colored_blob_camera_sensor_impl&) = default;

  /**
   * \brief Get the sensor readings for the footbot robot.
   *
   * \return A vector of \ref reading.
   */
  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_blob_camera_sensor<U>::value)>
  std::vector<reading>  readings(void) const {
    ER_ASSERT(nullptr != m_sensor,
              "%s called with NULL impl handle!",
              __FUNCTION__);

    std::vector<reading> ret;
    for (auto &r : m_sensor->GetReadings().BlobList) {
      struct reading s = {
        /* ARGoS reports this in cm for some reason */
        .vec = {r->Distance / 100.0,
                rmath::radians(r->Angle.GetValue())},
        .color = rutils::color(r->Color.GetRed(),
                              r->Color.GetGreen(),
                              r->Color.GetBlue())
      };
      ret.push_back(s);
    } /* for(&r..) */

    return ret;
  }

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_blob_camera_sensor<U>::value)>
  void enable(void) const {
    ER_ASSERT(nullptr != m_sensor,
              "%s called with NULL impl handle!",
              __FUNCTION__);
    m_sensor->Enable();
  }

  template <typename U = TSensor,
            RCPPSW_SFINAE_DECLDEF(detail::is_argos_blob_camera_sensor<U>::value)>
  void disable(void) const {
    ER_ASSERT(nullptr != m_sensor,
              "%s called with NULL impl handle!",
              __FUNCTION__);
    m_sensor->Disable();
  }

 private:
  /* clang-format off */
  TSensor* const m_sensor;
  /* clang-format on */
};

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using colored_blob_camera_sensor =
    colored_blob_camera_sensor_impl<argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;
#else
class colored_blob_camera_sensor{};
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(sensors, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SENSORS_COLORED_BLOB_CAMERA_SENSOR_HPP_ */
