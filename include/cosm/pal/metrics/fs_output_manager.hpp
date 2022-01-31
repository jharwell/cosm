/**
 * \file fs_output_manager.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_PAL_METRICS_FS_OUTPUT_MANAGER_HPP_
#define INCLUDE_COSM_PAL_METRICS_FS_OUTPUT_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/pal/pal.hpp"

#if defined(COSM_PAL_TARGET_ARGOS)
#include "cosm/pal/argos/metrics/fs_output_manager.hpp"
#elif defined(COSM_PAL_TARGET_ROS)
#include "cosm/pal/ros/metrics/fs_output_manager.hpp"
#else
#error Selected platform does not support metrics output to filesystem
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using fs_output_manager = cpargos::metrics::fs_output_manager;
#elif defined(COSM_PAL_TARGET_ROS)
using fs_output_manager = cpros::metrics::fs_output_manager;
#endif /* COSM_PAL_TARGET_ARGOS */

NS_END(metrics, pal, cosm);

#endif /* INCLUDE_COSM_PAL_METRICS_FS_OUTPUT_MANAGER_HPP_ */
