/**
 * \file dist_status.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::block_dist {

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The distribution status of \a ALL blocks that are distributed at once
 * via \ref dispatcher.
 */

enum class dist_status {
  /**
   * \brief All blocks/a given block were distributed successfully.
   */
  ekSUCCESS,

  /**
   * \brief Fatal failure to distribute one or more blocks.
   *
   */
  ekFAILURE
};

} /* namespace cosm::foraging::block_dist */

