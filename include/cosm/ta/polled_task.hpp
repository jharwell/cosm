/**
 * \file polled_task.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
  void exec_estimate_init(const rmath::rangez& bounds, rmath::rng* rng);

 private:
  std::unique_ptr<taskable> m_mechanism;
};

NS_END(ta, cosm);
