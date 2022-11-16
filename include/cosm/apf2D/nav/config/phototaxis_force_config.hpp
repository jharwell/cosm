/**
 * \file phototaxis_force_config.hpp
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct phototaxis_force_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration for virtual phototaxis force.
 */
struct phototaxis_force_config final : public rconfig::base_config {
  double max{0};
};

} /* namespace cosm::apf2D::nav::config */
