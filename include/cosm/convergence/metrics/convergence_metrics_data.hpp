/**
 * \file convergence_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct convergence_metrics_data_impl
 * \ingroup convergence metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * cconvergence::convergence_metrics.
 */
struct convergence_measure_data_impl {
  double raw{0.0};
  double norm{0.0};
  bool converged{false};
};

struct convergence_metrics_data : public rmetrics::base_data {
  double                           conv_epsilon{0.0};
  convergence_measure_data_impl interact{};
  convergence_measure_data_impl order{};
  convergence_measure_data_impl pos_ent{};
  convergence_measure_data_impl tdist_ent{};
  convergence_measure_data_impl velocity{};
};

} /* namespace cosm::convergence::metrics */

