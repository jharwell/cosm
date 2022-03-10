/**
 * \file taskable.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Class Decls
 ******************************************************************************/
class taskable_argument;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class taskable
 * \ingroup ta
 *
 * \brief A class that all classes wishing to be used as the mechanism by which
 * \ref executable_task objects execute themselves must inherit from.
 */
class RCPPSW_EXPORT taskable : public virtual rmetrics::base_metrics {
 public:
  taskable(void) = default;
  ~taskable(void) override;

  /**
   * \brief Execute the task.
   */
  virtual void task_execute(void) = 0;

  /**
   * \brief Determine if the task has finished yet.
   *
   * \return \c TRUE if the task has finished, and \c FALSE otherwise.
   */
  virtual bool task_finished(void) const = 0;

  /**
   * \brief Determine if the task has is still running yet.
   *
   * \return \c TRUE if the task is still running, and \c FALSE otherwise.
   */
  virtual bool task_running(void) const = 0;

  /**
   * \brief Reset the task so that it is ready for execution again. Does nothing
   * by default.
   */
  virtual void task_reset(void) {}

  /**
   * \brief Start the task with the specified argument. The argument is consumed
   * by the called function.
   */
  virtual void task_start(RCPPSW_UNUSED taskable_argument* c_arg) = 0;
};

NS_END(ta, cosm);

