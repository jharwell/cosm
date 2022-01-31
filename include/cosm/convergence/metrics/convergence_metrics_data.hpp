/**
 * \file convergence_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_DATA_HPP_
#define INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_DATA_HPP_

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

#endif /* INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_DATA_HPP_ */
