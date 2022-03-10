/**
 * \file bi_tdgraph_metrics.hpp
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
#include "rcppsw/common/common.hpp"
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_metrics
 * \ingroup ta metrics
 *
 * \brief Interface defining metrics that can be collected about the current
 * task dist of a \ref bi_tdgraph.
 */
class RCPPSW_EXPORT bi_tdgraph_metrics : public virtual rmetrics::base_metrics {
 public:
  bi_tdgraph_metrics(void) = default;
  ~bi_tdgraph_metrics(void) override = default;

  /**
   * \brief Return the depth of the curent task within the task decomposition
   * graph, or -1 if there is no current task.
   */
  virtual int current_task_depth(void) const = 0;

  /**
   * \brief Return a uuid for the current task, for use in calculating overall
   * task bi_tdgraph_dist, or -1 if no current task.
   */
  virtual int current_task_id(void) const = 0;

  /**
   * \brief Return a uuid for the TAB that the current task belongs to.
   */
  virtual int current_task_tab(void) const = 0;
};

NS_END(metrics, ta, cosm);

