/**
 * \file base_controller2D.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/base_controller2D.hpp"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <fstream>

#include "rcppsw/math/config/rng_config.hpp"
#include "rcppsw/math/rngm.hpp"

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"
#include "cosm/kin/metrics_proxy.hpp"
#include "cosm/kin/metrics/contexts.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller2D::base_controller2D(void) = default;

base_controller2D::~base_controller2D(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_controller2D::sensing_update(const rtypes::timestep& tick,
                                       const rtypes::discretize_ratio& ratio) {
  saa()->sensing()->update(tick, ratio);
} /* sensing_update() */

void base_controller2D::sensing_update(const rtypes::timestep& tick) {
  saa()->sensing()->update(tick);
} /* sensing_update() */

void base_controller2D::mdc_ts_update(void) const {
  ER_MDC_RM("time");

  auto tick = RCPPSW_LIKELY(nullptr != saa() && nullptr != saa()->sensing())
                  ? saa()->sensing()->tick()
                  : rtypes::timestep(0);
  ER_MDC_ADD("time", "[t=" + rcppsw::to_string(tick) + std::string("]"));
}

/*******************************************************************************
 * Swarm Spatial Metrics
 ******************************************************************************/
rmath::vector2d base_controller2D::rpos2D(void) const {
  return base_controller::rpos2D();
}

rmath::vector2z base_controller2D::dpos2D(void) const {
  return base_controller::dpos2D();
}

rmath::radians base_controller2D::heading2D(void) const {
  return base_controller::azimuth();
}

} /* namespace cosm::controller */
