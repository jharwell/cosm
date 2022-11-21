/**
 * \file task_id_extract.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller::operations {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct task_id_extract
 * \ingroup controller operations
 *
 * \brief Given a robot controller of type T, extract the ID of its current
 * task. This is used in computing task distribution entropy.
 */
template <class TController>
struct task_id_extract {
  task_id_extract(void) = default;

  int operator()(const TController* const c) const {
    return c->current_task_id();
  }
};

} /* namespace cosm::controller::operations */

