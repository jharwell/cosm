/**
 * \file movement_metrics_data.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

#include "cosm/spatial/metrics/movement_category.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct movement_metrics_data_data
 * \ingroup spatial metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * movement_metrics. Must be atomic so counts are valid in parallel metric
 * collection contexts. Ideally the distances would be atomic \ref
 * rtypes::spatial_dist, but that type does not meet the std::atomic
 * requirements.
 */
struct movement_metrics_data {
  ral::mt_double_t distance{0.0};
  ral::mt_size_t  n_robots{0};
  ral::mt_double_t velocity{0.0};
};

NS_END(detail);

struct movement_metrics_data : public rmetrics::base_data {
  std::vector<detail::movement_metrics_data> interval{rcppsw::as_underlying(movement_category::ekMAX)};
  std::vector<detail::movement_metrics_data> cum{rcppsw::as_underlying(movement_category::ekMAX)};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  movement_metrics_data& operator+=(const movement_metrics_data& rhs) {
    for (size_t i = 0; i < movement_category::ekMAX; ++i) {
      ral::mt_accum(this->interval[i].distance, rhs.interval[i].distance);
      ral::mt_accum(this->interval[i].n_robots, rhs.interval[i].n_robots);
      ral::mt_accum(this->interval[i].velocity, rhs.interval[i].velocity);

      ral::mt_accum(this->cum[i].distance, rhs.interval[i].distance);
      ral::mt_accum(this->cum[i].n_robots, rhs.interval[i].n_robots);
      ral::mt_accum(this->cum[i].velocity, rhs.interval[i].velocity);
    } /* for(i..) */
    return *this;
  }
};

NS_END(metrics, spatial, cosm);
