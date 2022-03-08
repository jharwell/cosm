/**
 * \file base_executive.hpp
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
#include <functional>
#include <list>
#include <memory>
#include <random>
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/ta/config/task_alloc_config.hpp"
#include "cosm/ta/ds/ds_variant.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);
namespace config {
struct task_executive_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_executive
 * \ingroup ta
 *
 * \brief Base class for runtime task task executives. Supports specification of
 * task allocation policy independent from the data structure storing
 * relationships among the tasks to be allocated (invalid combinations result in
 * compiler errors).
 */
class base_executive : public rer::client<base_executive> {
 public:
  using abort_notify_cb = std::function<void(polled_task*)>;
  using finish_notify_cb = std::function<void(polled_task*)>;
  using start_notify_cb = std::function<void(polled_task*)>;

  /**
   * \brief Creates the base executive.
   *
   * \param exec_config Initialization config specific to executive.
   * \param alloc_config Initialization config for task allocation.
   * \param ds Data structure containing tasks to manage/run.
   * \param rng Non-owning Random number generator reference.
   */
  base_executive(const config::task_executive_config* exec_config,
                 const config::task_alloc_config* alloc_config,
                 std::unique_ptr<ds::ds_variant> ds,
                 rmath::rng* rng);
  ~base_executive(void) override;

  base_executive& operator=(const base_executive&) = delete;
  base_executive(const base_executive&) = delete;

  /**
   * \brief The means by which the task executive will run one
   * timestep.
   */
  void run(void);

  const config::task_alloc_config* alloc_config(void) const {
    return &mc_alloc_config;
  }

  /**
   * \brief Get the task currently being run.
   */
  const polled_task* current_task(void) const { return m_current_task; }
  polled_task* current_task(void) { return m_current_task; }

  /**
   * \brief Set an optional callback that will be run when a task is aborted.
   *
   * The callback will be passed the task that was aborted, before the active
   * task is reset and after any time estimates have been updated on the aborted
   * task (the task is marked as aborted before calling).
   */
  void task_abort_notify(const abort_notify_cb& cb) {
    m_task_abort_notify.push_back(cb);
  }
  const std::list<abort_notify_cb>& task_abort_notify(void) const {
    return m_task_abort_notify;
  }

  /**
   * \brief Set an optional callback that will be run when after a task is
   * finished.
   *
   * The callback will be passed a pointer to the task that was just finished,
   * before the task is reset and after time estimates are updated on the
   * finished task. The task will have its execution and interface times updated
   * (if applicable) prior to the call.
   */
  void task_finish_notify(const finish_notify_cb& cb) {
    m_task_finish_notify.push_back(cb);
  }
  const std::list<finish_notify_cb>& task_finish_notify(void) const {
    return m_task_finish_notify;
  }

  /**
   * \brief Set an optional callback that will be run when a task is started.
   *
   * The callback will be passed a pointer to the task that was just started,
   * before the task is reset.
   */
  void task_start_notify(const start_notify_cb& cb) {
    m_task_start_notify.push_back(cb);
  }
  const std::list<start_notify_cb>& task_start_notify(void) const {
    return m_task_start_notify;
  }

  const ds::ds_variant* ds(void) const { return m_ds.get(); }
  bool update_exec_ests(void) const { return mc_update_exec_ests; }
  bool update_interface_ests(void) const { return mc_update_interface_ests; }

 protected:
  /**
   * \brief Handler called when a task is aborted.
   *
   * The base implementation does the following, in order:
   *
   * - Update exec/interface times, time estimates.
   * - Mark the task as aborted.
   * - Call the task abort callbacks with the aborted task.
   * - Mark the task as not aborted.
   * - Start a new task via \ref task_start_handle().
   */
  virtual void task_abort_handle(polled_task* task);

  /**
   * \brief Handler called to start a new task.
   *
   * The base implementation does the following, in order:
   *
   * - Call the task start callbacks for the new task.
   * - Call \ref do_task_start() for the new task.
   */
  virtual void task_start_handle(polled_task* new_task);

  /**
   * \brief Handler called when a task has been finished (not aborted).
   *
   * The base implementation does the following, in order:
   *
   * - Update task exec/interface times/estimates
   * - Call the task finish callbacks for the finished task.
   * - Start a new task via \ref task_start_handle().
   */
  virtual void task_finish_handle(polled_task* task);

  /**
   * \brief Allocate a new task, given the most recently executed
   * one.
   */
  virtual polled_task* task_allocate(const polled_task* task) = 0;

  /**
   * \brief Low-level start start handling:
   *
   * - Reset the task.
   * - Actually start the task.
   * - Set the current task for the executive to the started task.
   */
  void do_task_start(polled_task* task);

  /**
   * \brief Update execution and interface time estimates (if configured to do
   * so) for the specified task.
   */
  void task_ests_update(polled_task* task);

  /**
   * \brief Update execution and interface times for the specified task.
   */
  void task_times_update(polled_task* task);

  void current_task(polled_task* current_task) { m_current_task = current_task; }

  ds::ds_variant* ds(void) { return m_ds.get(); }

  const rmath::rng* rng(void) const { return m_rng; }
  rmath::rng* rng(void) { return m_rng; }

  uint task_alloc_count(void) const { return m_alloc_count; }

 private:
  /* clang-format off */
  const bool                      mc_update_exec_ests;
  const bool                      mc_update_interface_ests;
  const config::task_alloc_config mc_alloc_config;

  uint                            m_alloc_count{0};
  polled_task*                    m_current_task{nullptr};
  std::list<abort_notify_cb>      m_task_abort_notify{};
  std::list<finish_notify_cb>     m_task_finish_notify{};
  std::list<start_notify_cb>      m_task_start_notify{};
  std::unique_ptr<ds::ds_variant> m_ds;
  rmath::rng*                     m_rng;
  /* clang-format on */
};

NS_END(ta, cosm);

