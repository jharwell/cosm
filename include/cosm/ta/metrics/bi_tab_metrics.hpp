/**
 * \file bi_tab_metrics.hpp
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
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tab_metrics
 * \ingroup ta metrics
 *
 * \brief Interface defining metrics that can be collected about the task
 * allocation each robot performs.
 *
 */
class bi_tab_metrics : public virtual rmetrics::base_metrics {
 public:
  bi_tab_metrics(void) = default;
  ~bi_tab_metrics(void) override = default;

  /**
   * \brief This function should return if subtask1 is currently the active
   * task.
   *
   * \return Valid at any time.
   */
  virtual bool subtask1_active(void) const = 0;

  /**
   * \brief This function should return if subtask2 is currently the active
   * task.
   *
   * \return Valid at any time.
   */
  virtual bool subtask2_active(void) const = 0;

  /**
   * \brief This function should return if the root is currently the active
   * task.
   *
   * \return Valid at any time.
   */
  virtual bool root_active(void) const = 0;

  /**
   * \brief This function should return \c TRUE, if a robot has chosen to employ
   * task partitioning when allocating itself its next task.
   *
   * \return Valid only when \ref task_changed() returns \c TRUE.
   */
  virtual bool employed_partitioning(void) const = 0;

  /**
   * \brief This function should return \c TRUE if the task that has just been
   * allocated is different than the one that it executed last time (this is the
   * definition of changing allocation).
   *
   * \return Undefined unless \ref task_changed() returns \c TRUE.
   */
  virtual bool task_changed(void) const = 0;

  /**
   * \brief This function should return \c TRUE iff the task that has just
   * been allocated is a different depth in the \ref tdgraph than the previous
   * task.
   *
   * \return Undefined unless \ref task_changed() returns \c TRUE.
   */
  virtual bool task_depth_changed(void) const = 0;

  /**
   * \brief Return the current partitioning probability for the root task in the
   * TAB.
   */
  virtual double partition_prob(void) const = 0;

  /**
   * \brief Return the current subtask selection probability for the root task
   * in the TAB.
   */
  virtual double subtask_selection_prob(void) const = 0;
};

NS_END(metrics, ta, cosm);

