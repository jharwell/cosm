/**
 * \file execution_metrics.hpp
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
#include "cosm/ta/time_estimate.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class execution_metrics
 * \ingroup ta metrics
 *
 * \brief Interface defining metrics that can be collected on tasks as they are
 * executed.
 */
class execution_metrics : public virtual rmetrics::base_metrics {
 public:
  execution_metrics(void) = default;
  ~execution_metrics(void) override = default;

  /**
   * \brief If \c TRUE, then the robot is currently at a task interface for
   * this task.
   */
  virtual bool task_at_interface(void) const = 0;

    /**
   * \brief This function should return the execution time of the most recent
   * execution of this task. Execution time includes interface time.
   */
  virtual rtypes::timestep task_last_exec_time(void) const = 0;

  /**
   * \brief This function should return the interface time of the most recent
   * interface time for the task.
   */
  virtual rtypes::timestep task_last_interface_time(size_t i) const = 0;

  /**
   * \brief This function should return \c TRUE iff when the task has been
   * completed (not aborted), and only on the timestep on which it has done so.
   */
  virtual bool task_completed(void) const = 0;

  /**
   * \brief This function should return \c TRUE iff the task has been aborted
   * (not completed), and only on the timestep on which it has done so.
   */
  virtual bool task_aborted(void) const = 0;

  /**
   * \brief Return the current execution time estimate for a task. Execution
   * estimate should include interface time.
   */
  virtual const ta::time_estimate& task_exec_estimate(void) const = 0;

  /**
   * \brief Return the current interface time estimate for interface i for a
   * task.
   */
  virtual const ta::time_estimate& task_interface_estimate(size_t i) const = 0;

  virtual int task_last_active_interface(void) const = 0;
};

NS_END(metrics, ta, cosm);

