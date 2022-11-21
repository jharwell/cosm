/**
 * \file redist_governor.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/block_dist/redist_governor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
redist_governor::redist_governor(
    const config::block_redist_governor_config* const config)
    : ER_CLIENT_INIT("cosm.foraging.block_dist.redist_governor"),
      mc_config(*config),
      m_dist_status(mc_config.redistribute) {
  ER_INFO("Recurrence policy=%s,trigger=%s",
          mc_config.recurrence_policy.c_str(),
          mc_config.disable_trigger.c_str());
  ER_INFO("Initial redistribution status: %d", m_dist_status);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void redist_governor::update(const rtypes::timestep& t,
                             size_t blocks_collected,
                             bool convergence_status) {

  /* # blocks is always infinite/finite, depending on initial status */
  if (kDisableTriggerNone == mc_config.disable_trigger) {
    return;
  }
  /*
   * Can only be tripped once, so if already tripped avoid printing
   * diagnostic multiple times.
   */
  else if (kDisableTriggerTime == mc_config.disable_trigger) {
    if (t >= mc_config.timestep && m_dist_status) {
      ER_INFO("Redistribution disabled by trigger '%s': "
              "t=%zu,n_blocks=%zu,convergence=%d",
              kDisableTriggerTime.c_str(),
              t.v(),
              blocks_collected,
              convergence_status);
      m_dist_status = false;
    }
  } else if (kDisableTriggerBlockCount == mc_config.disable_trigger) {
    if (blocks_collected >= mc_config.block_count && m_dist_status) {
      ER_INFO("Redistribution disabled by '%s': "
              "t=%zu,n_blocks=%zu,convergence=%d",
              kDisableTriggerBlockCount.c_str(),
              t.v(),
              blocks_collected,
              convergence_status);
      m_dist_status = false;
    }
  } else if (kDisableTriggerConvergence == mc_config.disable_trigger) {
    if (kRecurrencePolicyLatch == mc_config.recurrence_policy &&
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
    ER_INFO("Redistribution=%d triggered by '%s': "
            "t=%zu,n_blocks=%zu,convergence=%d",
            m_dist_status,
            kDisableTriggerConvergence.c_str(),
            t.v(),
            blocks_collected,
            convergence_status);
    return;
  } else {
    ER_FATAL_SENTINEL("Bad trigger type '%s'",
                      mc_config.disable_trigger.c_str());
  }
} /* update() */

} /* namespace cosm::foraging::block_dist */
