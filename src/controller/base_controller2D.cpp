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
#include <experimental/filesystem>
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
namespace fs = std::experimental::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller2D::base_controller2D(void)
    : ER_CLIENT_INIT("cosm.controller2D.base"), m_saa(nullptr) {}

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

std::string base_controller2D::output_init(const std::string& output_root,
                                           const std::string& output_dir) {
  std::string dir;
  if ("__current_date__" == output_dir) {
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    dir = output_root + "/" + rcppsw::to_string(now.date().year()) + "-" +
          rcppsw::to_string(now.date().month()) + "-" +
          rcppsw::to_string(now.date().day()) + ":" +
          rcppsw::to_string(now.time_of_day().hours()) + "-" +
          rcppsw::to_string(now.time_of_day().minutes());
  } else {
    dir = output_root + "/" + output_dir;
  }

  if (!fs::exists(dir)) {
    fs::create_directories(dir);
  }

#if (LIBRA_ER == LIBRA_ER_ALL)
  /*
   * Each file appender is attached to a root category in the COSM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.controller2D"),
                 dir + "/controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.fsm"), dir + "/fsm.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.subsystem.saa"),
                 dir + "/saa.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.robots.footbot.saa"),
                 dir + "/saa.log");
#endif

  return dir;
} /* output_init() */

void base_controller2D::rng_init(int seed, const std::string& category) {
  rmath::rngm::instance().register_type<rmath::rng>(category);
  if (-1 == seed) {
    ER_INFO("Using time seeded RNG for category '%s'", category.c_str());
    m_rng = rmath::rngm::instance().create(
        category, std::chrono::system_clock::now().time_since_epoch().count());
  } else {
    ER_INFO("Using user seeded RNG for category '%s'", category.c_str());
    m_rng = rmath::rngm::instance().create(category, seed);
  }
} /* rng_init() */

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

rmath::vector2d base_controller2D::velocity(void) const {
  /*
   * If you allow distance gathering at timesteps < 1, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (saa()->sensing()->tick() > 1U) {
    return saa()->linear_velocity();
  }
  return {0, 0};
} /* velocity() */

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
const rmath::vector2d& base_controller2D::position2D(void) const {
  return m_saa->sensing()->position();
}
const rmath::vector2u& base_controller2D::discrete_position2D(void) const {
  return m_saa->sensing()->discrete_position();
}

rmath::vector2d base_controller2D::heading2D(void) const {
  return {1.0, m_saa->sensing()->heading()};
}

NS_END(controller, cosm);
