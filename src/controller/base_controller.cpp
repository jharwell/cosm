/**
 * \file base_controller.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
#include "cosm/kin/metrics_proxy.hpp"
#include "cosm/subsystem/base_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {

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
fs::path base_controller::output_init(const cpconfig::output_config* config) {
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

void base_controller::kin_proxy(std::unique_ptr<ckin::metrics_proxy> prox) {
  m_kin_proxy = std::move(prox);
} /* kin_proxy() */

void base_controller::saa(std::unique_ptr<subsystem::base_saa_subsystem> saa) {
  m_saa = std::move(saa);
} /* saa() */

rmath::vector3d base_controller::rpos3D(void) const {
  return m_saa->sensing()->rpos3D();
}

rmath::vector2d base_controller::rpos2D(void) const {
  return m_saa->sensing()->rpos2D();
}

rmath::vector3z base_controller::dpos3D(void) const {
  return m_saa->sensing()->dpos3D();
}

rmath::vector2z base_controller::dpos2D(void) const {
  return m_saa->sensing()->dpos2D();
}

rmath::radians base_controller::azimuth(void) const {
  return m_saa->sensing()->azimuth();
}

rmath::radians base_controller::zenith(void) const {
  return m_saa->sensing()->zenith();
}

} /* namespace cosm::controller */
