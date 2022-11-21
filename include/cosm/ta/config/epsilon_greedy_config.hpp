/**
 * \file epsilon_greedy_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct epsilon_greedy_config
 * \ingroup ta config
 *
 * \brief Configuration for the \f$\epsilon\f$-greedy task allocation method, as
 * described in \todo paper ref.
 */
struct epsilon_greedy_config final : public rcppsw::config::base_config {
  double epsilon{-1};
  std::string regret_bound{};
};

} /* namespace cosm::ta::config */

