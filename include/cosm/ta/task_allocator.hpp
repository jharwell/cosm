/**
 * \file task_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <string>

#include "rcppsw/common/common.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/ta/bi_tdgraph_allocator.hpp"
#include "cosm/ta/ds/ds_variant.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class task_allocator
 * \ingroup ta
 *
 * \brief Maps the task data structure to its variant, and then applies the
 * corresponding allocation policy to the mapped variant to allocate a task.
 */
class task_allocator : public boost::static_visitor<polled_task*> {
 public:
  task_allocator(const config::task_alloc_config* config, rmath::rng* rng)
      : m_config(config), m_rng(rng) {}

  task_allocator& operator=(const task_allocator&) = delete;
  task_allocator(const task_allocator&) = delete;

  polled_task* operator()(ds::bi_tdgraph& graph,
                          const polled_task* last_task,
                          size_t alloc_count) const {
    return bi_tdgraph_allocator(m_config, &graph, m_rng)(last_task, alloc_count);
  }

 private:
  /* clang-format off */
  const config::task_alloc_config* m_config;
  rmath::rng*                      m_rng;
  /* clang-format on */
};

} /* namespace cosm::ta */
