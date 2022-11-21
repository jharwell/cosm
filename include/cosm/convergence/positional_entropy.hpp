/**
 * \file positional_entropy.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::convergence {

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

} /* namespace cosm::convergence */
