/**
 * \file grid3D_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct grid3D_config
 * \ingroup config ds
 *
 * \brief Configuration for \ref rds::grid3D_overlay objects.
 */
struct grid3D_config final : public rconfig::base_config {
  rtypes::discretize_ratio resolution{0.0};
  rmath::vector3d dims{};
};

NS_END(config, ds, cosm);

