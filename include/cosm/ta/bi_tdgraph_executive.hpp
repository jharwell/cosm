/**
 * \file bi_tdgraph_executive.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <memory>
#include <string>

#include "rcppsw/rcppsw.hpp"

#include "cosm/ta/base_executive.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);
class bi_tdgraph;
struct executive_config;

namespace ds {
class bi_tab;
} /* namespace ds */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_executive
 * \ingroup ta
 *
 * \brief A task executive which tasks are run one step at a time and polled
 * until they are finished. Operates on \ref bi_tdgraph.
 */
class bi_tdgraph_executive final : public base_executive,
                                   public rer::client<bi_tdgraph_executive> {
 public:
  using start_notify_cb = std::function<void(polled_task*, const ds::bi_tab*)>;

  bi_tdgraph_executive(const config::task_executive_config* exec_config,
                       const config::task_alloc_config* const alloc_config,
                       ds::ds_variant&& ds,
                       rmath::rng* rng);

  /**
   * \brief Get the TAB corresponding to the currently active task.
   */
  const ds::bi_tab* active_tab(void) const RCPPSW_PURE;

  /**
   * \brief Set an optional callback that will be run when a new task is
   * started.
   *
   * The callback will be passed a pointer to the task that was just started.
   */
  void task_start_notify(const start_notify_cb& cb) {
    m_task_start_notify.push_back(cb);
  }

  const polled_task* root_task(void) const RCPPSW_PURE;

  /**
   * \brief Get the parent task of the specified one.
   */
  const polled_task* parent_task(const polled_task* v);

  const ds::bi_tdgraph* graph(void) const;

 protected:
  polled_task* root_task(void) RCPPSW_PURE;
  ds::bi_tdgraph* graph(void);

 private:
  polled_task* task_allocate(const polled_task* last_task) override;
  void task_start_handle(polled_task* new_task) override;
  void task_abort_handle(polled_task* task) override;
  void task_finish_handle(polled_task* task) override;

  void active_tab_update(void);
  /* clang-format off */
  std::list<start_notify_cb> m_task_start_notify{};
  /* clang-format on */
};

NS_END(ta, cosm);
