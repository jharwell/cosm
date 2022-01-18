/**
 * \file base_controller.cpp
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
#include "cosm/controller/base_controller.hpp"

#include <filesystem>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/fsm/supervisor_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller);
namespace fs = std::filesystem;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller::base_controller(void)
    : ER_CLIENT_INIT("cosm.controller.base") {}

base_controller::~base_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
fs::path base_controller::output_init(
    const cpconfig::output_config* config) {
  auto path = cpconfig::output_config::root_calc(config);

  if (!fs::exists(path)) {
    fs::create_directories(path);
  }

  /*
   * Each file appender is attached to a root category in the COSM
   * namespace. If you give different file appenders the same file, then the
   * lines within it are not always ordered, which is not overly helpful for
   * debugging.
   */
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.controller"),
                 path / "controller.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.fsm"), path / "fsm.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.subsystem.saa"),
                 path / "saa.log");
  ER_LOGFILE_SET(log4cxx::Logger::getLogger("cosm.robots.footbot.saa"),
                 path / "saa.log");
  return path;
} /* output_init() */

void base_controller::rng_init(int seed, const std::string& category) {
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

void base_controller::supervisor(std::unique_ptr<cfsm::supervisor_fsm> fsm) {
  m_supervisor = std::move(fsm);
} /* supervisor() */

void base_controller::inta_tracker(
    std::unique_ptr<cspatial::interference_tracker> inta) {
  m_inta_tracker = std::move(inta);
} /* inta_tracker() */

NS_END(controller, cosm);
