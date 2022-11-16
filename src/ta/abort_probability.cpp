/**
 * \file abort_probability.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/abort_probability.hpp"

#include <cmath>

#include "rcppsw/math/config/sigmoid_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
abort_probability::abort_probability(
    const rmath::config::sigmoid_config* const config)
    : sigmoid(config->reactivity, config->offset, config->gamma) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double abort_probability::operator()(const rtypes::timestep& exec_time,
                                     const time_estimate& whole_task) {
  if (!(whole_task.v() > 0)) {
    return eval(kMIN_ABORT_PROB);
  }
  double ratio = exec_time.v() / static_cast<double>(whole_task.v());
  double theta = 0.0;
  if (ratio <= offset()) {
    theta = reactivity() * (offset() - ratio);
  } else {
    theta = reactivity() * (ratio - offset());
  }
  return eval(1.0 / (1 + std::exp(theta)) * gamma());
} /* operator() */

NS_END(ta, cosm);
