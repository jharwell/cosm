/**
 * \file redist_governor.cpp
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
#include "cosm/foraging/block_dist/redist_governor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
redist_governor::redist_governor(
    const config::block_redist_governor_config* const config)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.redist_governor"),
      mc_config(*config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void redist_governor::update(const rtypes::timestep& t,
                             size_t blocks_collected,
                             bool convergence_status) {
  /* # blocks is always infinite */
  if (kTriggerNull == mc_config.trigger) {
    return;
  }
  /*
   * Can only be tripped once, so if already tripped avoid printing
   * diagnostic multiple times.
   */
  else if (kTriggerTime == mc_config.trigger) {
    if (t >= mc_config.timestep && m_dist_status) {
      ER_INFO(
          "Redistribution disabled by trigger '%s': "
          "t=%u,n_blocks=%zu,convergence=%d",
          kTriggerTime,
          t.v(),
          blocks_collected,
          convergence_status);
      m_dist_status = false;
    }
  } else if (kTriggerBlockCount == mc_config.trigger) {
    if (blocks_collected >= mc_config.block_count && m_dist_status) {
      ER_INFO("Redistribution disabled by '%s': t=%u,n_blocks=%zu,convergence=%d",
              kTriggerBlockCount,
              t.v(),
              blocks_collected,
              convergence_status);
      m_dist_status = false;
    }
  } else if (kTriggerConvergence == mc_config.trigger) {
    if (kStatusSwitchPolicySingle == mc_config.recurrence_policy &&
        !m_dist_status) {
      return;
    }
    /*
     * For multi switch, we only redistribute blocks when the swarm is not
     * converged.
     */
    if (m_dist_status == !convergence_status) {
      return;
    }
    m_dist_status = !convergence_status;
    ER_INFO(
        "Redistribution=%d triggered by '%s': t=%u,n_blocks=%zu,convergence=%d",
        m_dist_status,
        kTriggerConvergence,
        t.v(),
        blocks_collected,
        convergence_status);
    return;
  } else {
    ER_FATAL_SENTINEL("Bad trigger type '%s'", mc_config.trigger.c_str());
  }
} /* update() */

NS_END(block_dist, foraging, cosm);
