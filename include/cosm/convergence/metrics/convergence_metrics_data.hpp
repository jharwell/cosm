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
NS_START(cosm, convergence, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct convergence_metrics_data_impl
 * \ingroup convergence metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * convergence_metrics.
 */
struct convergence_measure_data {
  double raw{0.0};
  double norm{0.0};
  bool converged{false};
};

NS_END(detail);

struct convergence_metrics_data : public rmetrics::base_data {
  double                           conv_epsilon{0.0};
  detail::convergence_measure_data interact{};
  detail::convergence_measure_data order{};
  detail::convergence_measure_data pos_ent{};
  detail::convergence_measure_data tdist_ent{};
  detail::convergence_measure_data velocity{};
};

NS_END(metrics, convergence, cosm);

