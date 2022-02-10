/**
 * \file task_dist_entropy.hpp
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
#include <algorithm>
#include <vector>

#include "rcppsw/math/ientropy.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/convergence/convergence_measure.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_dist_entropy
 * \ingroup convergence
 *
 * \brief Calculate the task_dist entropy of the swarm, using the methods
 * outlined in Balch2000 and Turgut2008.
 */
class task_dist_entropy final : public convergence_measure {
 public:
  explicit task_dist_entropy(double epsilon) : convergence_measure(epsilon) {}

  /**
   * \brief Calculate the task distribution entropy of the swarm.
   *
   * \param tasks The current task distribution (each task is a unique
   *              non-negative integer).
   */
  bool operator()(const std::vector<int>& tasks) {
    int n_tasks =
        *std::max_element(tasks.begin(), tasks.end()) + 1; /* 0-based indexing */

    if (-1 == n_tasks) { /* no controller have active tasks */
      return false;
    }
    /* Count occurences of each task to obtain a discretized distribution */
    std::vector<int> accum(n_tasks);
    for (auto& t : tasks) {
      if (-1 != t) { /* -1 indicates no active task */
        accum[t]++;
      }
    } /* for(&t..) */

    /*
     * Divide the # of occurrencies of each task by the total # of elements in
     * the distribution in order to obtain the proportional representation
     * needed as input into the informational entropy calculation.
     */
    std::vector<double> dist(n_tasks);
    for (size_t i = 0; i < dist.size(); ++i) {
      dist[i] = static_cast<double>(accum[i]) / tasks.size();
    } /* for(i..) */

    update_raw(rmath::ientropy()(dist));
    set_norm(rmath::normalize(raw_min(), raw_max(), raw()));
    return update_convergence_state();
  }
};

NS_END(convergence, cosm);

