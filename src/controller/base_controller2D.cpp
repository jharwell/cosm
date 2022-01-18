/**
 * \file base_controller2D.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controller2D.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller2D::base_controller2D(void) : m_saa(nullptr) {}

base_controller2D::~base_controller2D(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_controller2D::sensing_update(const rtypes::timestep& tick,
                                       const rtypes::discretize_ratio& ratio) {
  m_saa->sensing()->update(tick, ratio);
} /* sensing_update() */

void base_controller2D::saa(std::unique_ptr<subsystem::saa_subsystemQ3D> saa) {
  m_saa = std::move(saa);
} /* saa() */

void base_controller2D::mdc_ts_update(void) const {
  ER_MDC_RM("time");

  auto tick = RCPPSW_LIKELY(nullptr != m_saa && nullptr != m_saa->sensing()) ?
              m_saa->sensing()->tick() : rtypes::timestep(0);
  ER_MDC_ADD("time",
             "[t=" +
             rcppsw::to_string(tick) +
             std::string("]"));
}

/*******************************************************************************
 * Movement
 ******************************************************************************/
rtypes::spatial_dist base_controller2D::ts_distance_impl(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa()->sensing()->tick() > 1U) {
    return rtypes::spatial_dist(saa()->sensing()->tick_travel2D().length());
  }
  return rtypes::spatial_dist(0.0);
} /* ts_distance_impl() */

rmath::vector3d base_controller2D::ts_velocity_impl(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (RCPPSW_LIKELY(saa()->sensing()->tick() > 1U)) {
    return saa()->odometry().twist.linear;
  }
  return { 0.0, 0.0, 0.0 };
} /* ts_velocity_impl() */

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
rmath::vector2d base_controller2D::rpos2D(void) const {
  return m_saa->sensing()->rpos2D();
}

rmath::vector2z base_controller2D::dpos2D(void) const {
  return m_saa->sensing()->dpos2D();
}

rmath::radians base_controller2D::heading2D(void) const {
  return m_saa->sensing()->heading();
}

NS_END(controller, cosm);
