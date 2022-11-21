/**
 * \file bi_tdgraph_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::ta::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_metrics
 * \ingroup ta metrics
 *
 * \brief Interface defining metrics that can be collected about the current
 * task dist of a \ref cta::ds::bi_tdgraph.
 */
class bi_tdgraph_metrics : public virtual rmetrics::base_metrics {
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
   * \brief Return a uuid for the TAB that the current task belongs to, or -1 if
   * no current TAB.
   */
  virtual int current_task_tab(void) const = 0;
};

} /* namespace cosm::ta::metrics */
