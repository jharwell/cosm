/**
 * \file task_dist_entropy.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::convergence {

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

} /* namespace cosm::convergence */
