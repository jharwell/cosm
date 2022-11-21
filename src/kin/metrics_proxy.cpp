/**
 * \file metrics_proxy.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin/metrics_proxy.hpp"

#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
boost::optional<rspatial::euclidean_dist>
metrics_proxy::traveled(const rmetrics::context& ctx) const {
  if (!mc_ctx_cb(ctx)) {
    return boost::none;
  }
  /*
   * If you allow distance gathering at timesteps < 2, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (RCPPSW_LIKELY(mc_sensing->tick() > 2U)) {
    auto ret = rspatial::euclidean_dist(mc_sensing->tick_travel3D().length());
    return boost::make_optional(ret);
  }
  return boost::none;
}

boost::optional<ckin::twist>
metrics_proxy::twist(const rmetrics::context& ctx) const {
  if (!mc_ctx_cb(ctx)) {
    return boost::none;
  }
  /*
   * If you allow distance gathering at timesteps < 2, you get a big jump
   * because of the prev/current location not being set up properly yet.
   */
  if (RCPPSW_LIKELY(mc_sensing->tick() > 2U)) {
    return boost::make_optional(mc_sensing->odometry()->reading().twist);
  }
  return boost::none;
}

ckin::pose metrics_proxy::pose(void) const {
  return mc_sensing->odometry()->reading().pose;
} /* pose() */

} /* namespace cosm::kin */
