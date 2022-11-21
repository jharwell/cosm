/**
 * \file base_controllerQ3D.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controllerQ3D.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controllerQ3D::base_controllerQ3D(void) = default;

base_controllerQ3D::~base_controllerQ3D(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_controllerQ3D::sensing_update(const rtypes::timestep& tick,
                                        const rtypes::discretize_ratio& ratio) {
  saa()->sensing()->update(tick, ratio);
} /* sensing_update() */

void base_controllerQ3D::sensing_update(const rtypes::timestep& tick) {
  saa()->sensing()->update(tick);
} /* sensing_update() */

void base_controllerQ3D::mdc_ts_update(void) const {
  ER_MDC_RM("time");
  auto tick = RCPPSW_LIKELY(nullptr != saa() && nullptr != saa()->sensing())
                  ? saa()->sensing()->tick()
                  : rtypes::timestep(0);
  ER_MDC_ADD("time", "[t=" + rcppsw::to_string(tick) + std::string("]"));
}

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
rmath::vector3d base_controllerQ3D::rpos3D(void) const {
  return base_controller::rpos3D();
}

rmath::vector2d base_controllerQ3D::rpos2D(void) const {
  return base_controller::rpos2D();
}

rmath::vector3z base_controllerQ3D::dpos3D(void) const {
  return base_controller::dpos3D();
}

rmath::vector2z base_controllerQ3D::dpos2D(void) const {
  return base_controller::dpos2D();
}

rmath::radians base_controllerQ3D::azimuth(void) const {
  return base_controller::azimuth();
}

rmath::radians base_controllerQ3D::heading2D(void) const { return azimuth(); }

rmath::radians base_controllerQ3D::zenith(void) const {
  return base_controller::zenith();
}

} /* namespace cosm::controller */
