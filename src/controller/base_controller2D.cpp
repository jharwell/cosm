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
#include <filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/subsystem/config/actuation_subsystem2D_config.hpp"
#include "cosm/subsystem/config/sensing_subsystem2D_config.hpp"
#include "cosm/subsystem/saa_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller);
namespace fs = std::filesystem;

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

void base_controller2D::saa(std::unique_ptr<subsystem::saa_subsystem2D> saa) {
  m_saa = std::move(saa);
} /* saa() */

#if (LIBRA_ER >= LIBRA_ER_ALL)
void base_controller2D::ndc_pusht(void) const {
  ER_NDC_PUSH("[t=" + rcppsw::to_string(m_saa->sensing()->tick()) +
              std::string("] [ent") + rcppsw::to_string(entity_id()) +
              std::string("]"));
}
#endif

/*******************************************************************************
 * Movement Metrics
 ******************************************************************************/
rtypes::spatial_dist base_controller2D::distance(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa()->sensing()->tick() > 1U) {
    return rtypes::spatial_dist(saa()->sensing()->tick_travel().length());
  }
  return rtypes::spatial_dist(0.0);
} /* distance() */

rmath::vector3d base_controller2D::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (RCSW_LIKELY(saa()->sensing()->tick() > 1U)) {
    auto vel = saa()->linear_velocity();
    return {vel.x(), vel.y(), 0.0};
  }
  return {0.0, 0.0, 0.0};
} /* velocity() */

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
rmath::vector2d base_controller2D::pos2D(void) const {
  return m_saa->sensing()->position();
}

rmath::vector2u base_controller2D::dpos2D(void) const {
  return m_saa->sensing()->discrete_position();
}

rmath::radians base_controller2D::heading2D(void) const {
  return m_saa->sensing()->heading();
}

NS_END(controller, cosm);
