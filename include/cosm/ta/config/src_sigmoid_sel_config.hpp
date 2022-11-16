/**
 * \file src_sigmoid_sel_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "rcppsw/config/base_config.hpp"
#include "cosm/ta/config/sigmoid_sel_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct src_sigmoid_sel_config
 * \ingroup config ta
 *
 * \brief Configuration for sigmoid derived probabilities to choose between
 * multiple options, where the sigmoid input source is changeable.
 */
struct src_sigmoid_sel_config final : public rcppsw::config::base_config {
  std::string input_src{};
  sigmoid_sel_config sigmoid{};
};

NS_END(config, ta, cosm);

