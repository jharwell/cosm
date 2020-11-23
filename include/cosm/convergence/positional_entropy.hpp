/**
 * \file positional_entropy.hpp
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

#ifndef INCLUDE_COSM_CONVERGENCE_POSITIONAL_ENTROPY_HPP_
#define INCLUDE_COSM_CONVERGENCE_POSITIONAL_ENTROPY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

#include "rcppsw/algorithm/clustering/entropy.hpp"
#include "rcppsw/algorithm/clustering/entropy_eh_omp.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/convergence/config/positional_entropy_config.hpp"
#include "cosm/convergence/convergence_measure.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class positional_entropy
 * \ingroup convergence
 *
 * \brief Calculate the positional entropy of the swarm, using the methods
 * outlined in Balch2000 and Turgut2008.
 */
class positional_entropy final
    : public convergence_measure,
      public raclustering::entropy_balch2000<rmath::vector2d> {
 public:
  positional_entropy(
      double epsilon,
      std::unique_ptr<raclustering::entropy_eh_omp<rmath::vector2d>> impl,
      const config::positional_entropy_config* const config)
      : convergence_measure(epsilon),
        entropy_balch2000(std::move(impl),
                          config->horizon,
                          config->horizon_delta) {}

  using entropy_balch2000::entropy_balch2000;

  /**
   * \brief Calculate the positional entropy in 2D space of a swarm.
   */
  bool operator()(const std::vector<rmath::vector2d>& data) {
    auto dist_func = [](const rmath::vector2d& v1, const rmath::vector2d& v2) {
      return (v1 - v2).length();
    };
    update_raw(run(data, dist_func));
    set_norm(rmath::normalize(raw_min(), raw_max(), raw()));
    return update_convergence_state();
  }
};

NS_END(convergence, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_POSITIONAL_ENTROPY_HPP_ */
