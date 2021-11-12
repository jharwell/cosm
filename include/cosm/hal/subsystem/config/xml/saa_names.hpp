/**
 * \file saa_names.hpp
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_XML_SAA_NAMES_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_XML_SAA_NAMES_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/common/common.hpp"
#include "cosm/hal/hal.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/config/xml/saa_names.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, subsystem, config, xml);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using saa_names = chargos::config::xml::saa_names;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(xml, config, subystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_CONFIG_XML_SAA_NAMES_HPP_ */
