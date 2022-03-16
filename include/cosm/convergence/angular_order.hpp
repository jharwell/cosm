/**
 * \file angular_order.hpp
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
#include <vector>

#include "rcppsw/math/radians.hpp"
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
 * \class angular_order
 * \ingroup convergence
 *
 * \brief Calculates the angular order within a swarm for a given instant. From
 * Turgut2008.
 */
class angular_order final : public convergence_measure {
 public:
  explicit angular_order(double epsilon) : convergence_measure(epsilon) {}

  /**
   * \brief Calculates the raw and normalized angular order for the swarm (from
   * Turgut2008).
   *
   * \return \c TRUE iff convergence has been achieved according to configured
   * parameters and the current state of the swarm.
   */
  bool operator()(const std::vector<rmath::radians>& headings,
                  RCPPSW_UNUSED size_t n_threads) {
    double y = 0.0;
    double x = 0.0;

#pragma omp parallel for num_threads(n_threads)
    for (auto it = headings.begin(); it < headings.end(); ++it) {
      y += std::sin((*it).v());
      x += std::cos((*it).v());
    } /* for(it..) */
    update_raw(std::fabs(std::atan2(y, x)) / headings.size());
    set_norm(rmath::normalize(raw_min(), raw_max(), raw()));
    return update_convergence_state();
  }
};

NS_END(convergence, cosm);
