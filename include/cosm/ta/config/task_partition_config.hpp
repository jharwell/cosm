/**
 * \file task_partition_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/ta/config/src_sigmoid_sel_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct task_partition_config
 * \ingroup config ta
 *
 * \brief Configuration for task partitioning probability calculation, as
 * described in \todo paper ref.
 */
struct task_partition_config final : public rcppsw::config::base_config {
  src_sigmoid_sel_config src_sigmoid{};
  bool always_partition{false};
  bool never_partition{false};
};

} /* namespace cosm::ta::config */

