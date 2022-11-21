/**
 * \file flocking_config.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/flocking/config/alignment_force_config.hpp"
#include "cosm/apf2D/flocking/config/constant_speed_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct flocking_config
 * \ingroup apf2D flocking config
 *
 * \brief Configuration force APF forces related to flocking.
 */
struct flocking_config : public rconfig::base_config {
  /* clang-format off */
  alignment_force_config            alignment{};
  constant_speed_force_config       constant_speed{};
  /* clang-format on */
};

} /* namespace cosm::apf2D::flocking::config */
