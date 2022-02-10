/**
 * \file velocity.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#include "rcppsw/math/vector2.hpp"
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
 * \class velocity
 * \ingroup convergence
 *
 * \brief Calculates the swarm velocity by finding its geometric average and
 * tracking how much that moves over time. From Turgut2008.
 */
class velocity final : public convergence_measure {
 public:
  explicit velocity(double epsilon) : convergence_measure(epsilon) {}

  /*
   * \brief Compute the velocity.
   */
  bool operator()(const std::vector<rmath::vector2d>& locs) {
    rmath::vector2d center =
        std::accumulate(locs.begin(), locs.end(), rmath::vector2d()) /
        locs.size();
    update_raw((center - m_prev_center).length());
    set_norm(rmath::normalize(raw_min(), raw_max(), raw()));
    m_prev_center = center;

    return update_convergence_state();
  }

 private:
  /* clang-format off */
  rmath::vector2d m_prev_center{};
  /* clang-format on */
};

NS_END(convergence, cosm);

