/**
 * \file interactivity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include <numeric>
#include <vector>

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
 * \class interactivity
 * \ingroup convergence
 *
 * \brief Calculates the degree of interaction of the swarm, given a list of
 * robot nearest neighbor distances from a swarm for a given instant. From
 * Szabo2014.
 *
 */
class interactivity final : public convergence_measure {
 public:
  explicit interactivity(double epsilon) : convergence_measure(epsilon) {}

  /*
   * \brief Compute the interaction degree.
   *
   * Note that each robot's distance to closest neighbor does not necessarily
   * appear in the same place in the result array on subsequent timesteps. This
   * is OK, because we are doing a cumulative addition with the results as we
   * collect metrics, so it doesn't really matter that you are not doing
   * strictly piecewise addition.
   */

  bool operator()(const std::vector<double>& dists) {
    update_raw(std::accumulate(dists.begin(), dists.end(), 0.0));

    /*
     * The 1.0 - factor is because if the raw degree is HIGHER than any we have
     * yet seen, that means that the swarm the LEAST interactive we have yet
     * seen it, and so should receive a value of 0.
     */
    set_norm(1.0 - rmath::normalize(raw_min(), raw_max(), raw()));
    return update_convergence_state();
  }
};

NS_END(convergence, cosm);
