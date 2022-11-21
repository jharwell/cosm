/**
 * \file base_controller3D.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
  */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controller3D.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/saa_subsystem3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller3D::base_controller3D(void) = default;

base_controller3D::~base_controller3D(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_controller3D::sensing_update(const rtypes::timestep& tick,
                                        const rtypes::discretize_ratio& ratio) {
  saa()->sensing()->update(tick, ratio);
} /* sensing_update() */

void base_controller3D::sensing_update(const rtypes::timestep& tick) {
  saa()->sensing()->update(tick);
} /* sensing_update() */

void base_controller3D::mdc_ts_update(void) const {
  ER_MDC_RM("time");
  auto tick = RCPPSW_LIKELY(nullptr != saa() && nullptr != saa()->sensing())
                  ? saa()->sensing()->tick()
                  : rtypes::timestep(0);
  ER_MDC_ADD("time", "[t=" + rcppsw::to_string(tick) + std::string("]"));
}

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
rmath::vector3d base_controller3D::rpos3D(void) const {
  return base_controller::rpos3D();
}

rmath::vector3z base_controller3D::dpos3D(void) const {
  return base_controller::dpos3D();
}

rmath::radians base_controller3D::azimuth(void) const {
  return base_controller::azimuth();
}

rmath::radians base_controller3D::zenith(void) const {
  return base_controller::zenith();
}

} /* namespace cosm::controller */
