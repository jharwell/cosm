/**
 * \file temporal_penalty_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/control/config/waveform_config.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct temporal_penalty_config
 * \ingroup tv config
 *
 * \brief Configuration for various types of temporal penalties which can be
 * applied to aspects of robot behavior.
 */
struct temporal_penalty_config final : public rconfig::base_config {
  /**
   * \brief If \c TRUE, then all instances of this penalty class will be
   * adjusted so that at most 1 robot finishes serving a penalty each timestep
   * (may not be applicable to all penalty types).
   */
  bool unique_finish{true};
  rct::config::waveform_config waveform{};
};

NS_END(tv, config, cosm);
