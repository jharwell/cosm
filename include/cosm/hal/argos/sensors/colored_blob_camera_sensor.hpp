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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <boost/optional.hpp>
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>

#include <argos3/plugins/robots/generic/control_interface/ci_colored_blob_omnidirectional_camera_sensor.h>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/utils/color.hpp"

#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"
#include "cosm/hal/argos/sensors/argos_sensor.hpp"
#include "cosm/hal/sensors/colored_blob_camera_sensor_reading.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors, detail);

/*******************************************************************************
 * Templates
 ******************************************************************************/
template<typename TSensor>
using is_blob_camera_sensor = std::is_same<
  TSensor,
  ::argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class colored_blob_camera_sensor_impl
 * \ingroup hal argos sensors
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
 *                  HAL. If nullptr, then that effectively disables the sensor.
 */
template <typename TSensor>
class colored_blob_camera_sensor_impl final : public rer::client<colored_blob_camera_sensor_impl<TSensor>>,
                                              public chargos::sensors::argos_sensor<TSensor> {
 private:
  using chargos::sensors::argos_sensor<TSensor>::decoratee;

 public:
  using impl_type = TSensor;
  using reading_type = chsensors::colored_blob_camera_sensor_reading;
  using chargos::sensors::argos_sensor<impl_type>::enable;
  using chargos::sensors::argos_sensor<impl_type>::disable;
  using chargos::sensors::argos_sensor<impl_type>::reset;
  using chargos::sensors::argos_sensor<impl_type>::is_enabled;

  explicit colored_blob_camera_sensor_impl(impl_type * const sensor)
      : ER_CLIENT_INIT("cosm.hal.sensors.colored_blob_camera"),
        chargos::sensors::argos_sensor<impl_type>(sensor) {
        disable();
  }

  /* move only constructible/assignable for use with saa subsystem */
  const colored_blob_camera_sensor_impl& operator=(const colored_blob_camera_sensor_impl&) = delete;
  colored_blob_camera_sensor_impl(const colored_blob_camera_sensor_impl&) = delete;
   colored_blob_camera_sensor_impl& operator=(colored_blob_camera_sensor_impl&&) = default;
  colored_blob_camera_sensor_impl(colored_blob_camera_sensor_impl&&) = default;

  /**
   * \brief Get the sensor readings for the robot.
   *
   * \param ref The angle of the reference frame to use (i.e., the robot's
   *            current heading).
   *
   * \return A vector of \ref reading_type.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_blob_camera_sensor<U>::value)>
  std::vector<reading_type>  readings(
      const rmath::radians& rframe = rmath::radians::kZERO) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    std::vector<reading_type> ret;
    for (auto &r : decoratee()->GetReadings().BlobList) {
      reading_type s = {
        /*
         * ARGoS reports the distance in cm for some reason, so put it in SI
         * units (meters), like a sane person.
         *
         * The angle reported is NOT relative to the global reference frame, but
         * to the robot's current heading, which is treated as if it is along
         * the positive X axis.
         */
        .vec = {r->Distance / 100.0,
                rframe + rmath::radians(r->Angle.GetValue())},
        .color = rutils::color(r->Color.GetRed(),
                              r->Color.GetGreen(),
                              r->Color.GetBlue())
      };
      ret.push_back(s);
    } /* for(&r..) */

    return ret;
  }

  /**
   * \brief Return the average blob reading within proximity for the
   * robot of the specified color.
   */
  template <typename U = impl_type,
            RCPPSW_SFINAE_DECLDEF(detail::is_blob_camera_sensor<U>::value)>
  boost::optional<rmath::vector2d> closest_blob(const rutils::color& color,
                                                const rmath::radians& rframe) const {
    ER_ASSERT(nullptr != decoratee(),
              "%s called with NULL impl handle!",
              __FUNCTION__);
    ER_ASSERT(is_enabled(),
              "%s called when disabled",
              __FUNCTION__);

    auto same_color = [&](auto& r) { return color == r.color; };

    /* constructed, not returned, so we need a copy to iterate over */
    auto readings = this->readings(rframe);
    auto filtered = readings | boost::adaptors::filtered(same_color);

    auto closest = boost::range::min_element(filtered,
                                             [&](const auto& r1, const auto& r2) {
                                               return r1.vec.length() < r2.vec.length();
                                             });

    if (closest == filtered.end()) {
      return boost::none;
    } else {
      return boost::make_optional(closest->vec);
    }
  }
};

using colored_blob_camera_sensor =
    colored_blob_camera_sensor_impl<::argos::CCI_ColoredBlobOmnidirectionalCameraSensor>;

NS_END(sensors, argos, hal, cosm);
