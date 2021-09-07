/**
 * \file polled_task.hpp
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

#ifndef INCLUDE_COSM_TA_POLLED_TASK_HPP_
#define INCLUDE_COSM_TA_POLLED_TASK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <utility>

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/ta/executable_task.hpp"
#include "cosm/ta/taskable.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class polled_task
 * \ingroup ta
 *
 * \brief Represents a task whose execution can/should be monitored by the user
 * to determine when it has finished.
 */
class polled_task : public executable_task, public taskable {
 public:
  polled_task(const std::string& name,
              const config::src_sigmoid_sel_config* abort,
              const rmath::config::ema_config* estimation,
              std::unique_ptr<taskable> mechanism)
      : executable_task(name, abort, estimation),
        m_mechanism(std::move(mechanism)) {}
  ~polled_task(void) override;

  polled_task& operator=(const polled_task&) = delete;
  polled_task(const polled_task&) = delete;

  taskable* mechanism(void) const { return m_mechanism.get(); }

  void task_execute(void) override final { m_mechanism->task_execute(); }
  void task_reset(void) override final { m_mechanism->task_reset(); }
  bool task_running(void) const override { return m_mechanism->task_running(); }
  bool task_finished(void) const override final {
    return m_mechanism->task_finished();
  }

  /**
   * \brief Initialize the execution time estimates of the task randomly within
   * the specified range.
   */
  void exec_estimate_init(const rmath::rangeu& bounds, rmath::rng* rng);

 private:
  std::unique_ptr<taskable> m_mechanism;
};

NS_END(ta, cosm);

#endif /* INCLUDE_COSM_TA_POLLED_TASK_HPP_ */
