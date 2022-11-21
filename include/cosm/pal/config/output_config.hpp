/**
 * \file output_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::config {

namespace fs = std::filesystem;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct output_config
 * \ingroup pal config
 *
 * \brief Configuration for robot/swarm manager output.
 */
struct output_config final : public rconfig::base_config {
  /**
   * \brief Absolute or relative path to the parent directory of the output root
   *        for the swarm manager/robot.
   */
  fs::path                 output_parent{};

  /**
   * \brief Directory name within the output parent that things should be output
   * into. This is a separate argument than output_root, because there are
   * special values of it that have different behavior.
   *
   * __current_date__ will cause \p output_leaf to be set to the current date in
   * the format "Y-M-D:H-M".
   */
  std::string              output_leaf{};

  rmconfig::metrics_config metrics {};

  static fs::path root_calc(const output_config* config);
};

} /* namespace cosm::pal::config */
