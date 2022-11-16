/**
 * \file redist_governor.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/cosm.hpp"
#include "cosm/foraging/config/block_redist_governor_config.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class redist_governor
 * \ingroup foraging block_dist
 *
 * \brief Supervises block re-distribution.
 *
 * Determinies the conditions under which blocks should be re-distributed after
 * being processed in a nest, if the # of blocks in the arena should become
 * finite, and whether or not that change is permanent.
 *
 * Conditions for disabling block re-distribution (and therefore the # blocks
 * becoming finite) are one of:
 *
 * - Null: the # blocks is always infinite/finite, depending what state the
 *   governor is initialized with.
 * - A specified # of blocks have been collected (any type).
 * - The swarm has converged, according to some measure.
 * - A specified number of timesteps has elapsed.
 *
 * Policies for determining if the switch to finite blocks is permanent or not
 * are:
 *
 * - latch -> it is permanent; once the trigger condition signal goes "high" it
 *   is remembered and if the signal goes "low" later that change is ignored.
 *
 * - multi -> it is not permanent: if the trigger condition goes "low" after
 *   being initially triggered by going "high", then block re-distribution is
 *   re-enabled until the trigger condition goes "high" again.
 */
class redist_governor : public rer::client<redist_governor> {
 public:
  static inline const std::string kDisableTriggerNull = "Null";
  static inline const std::string kDisableTriggerTime = "timestep";
  static inline const std::string kDisableTriggerBlockCount = "block_count";
  static inline const std::string kDisableTriggerConvergence = "convergence";
  static inline const std::string kRecurrencePolicyLatch = "latch";
  static inline const std::string kRecurrencePolicyMulti = "multi";

  explicit redist_governor(const config::block_redist_governor_config* config);

  /**
   * \brief Update the distribution status according to the policy parameters.
   *
   * \param t Current timestep.
   * \param blocks_collected # blocks collected so far.
   * \param convergence_status Current swarm convergence status.
   */
  void update(const rtypes::timestep& t,
              size_t blocks_collected,
              bool convergence_status);

  bool enabled(void) const { return m_dist_status; }

 private:
  /* clang-format off */
  const config::block_redist_governor_config mc_config;

  bool                                       m_dist_status;
  /* clang-format on */
};

NS_END(block_dist, foraging, cosm);
