/**
 * \file base_sm_repository.hpp
 *
 * Handles parsing of all XML parameters at runtime.
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_CONFIG_XML_BASE_SM_REPOSITORY_HPP_
#define INCLUDE_COSM_PAL_CONFIG_XML_BASE_SM_REPOSITORY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/xml/xml_config_repository.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_sm_repository
 * \ingroup pal config xml
 *
 * \brief Collection of all XML parsers and parse results common to all \ref
 * swarm_manager derived classes.
 */
class base_sm_repository : public rconfig::xml::xml_config_repository {
 public:
  base_sm_repository(void) noexcept RCPPSW_COLD;
};

NS_END(xml, config, pal, cosm);

#endif /* INCLUDE_COSM_PAL_CONFIG_XML_BASE_SM_REPOSITORY_HPP_ */
