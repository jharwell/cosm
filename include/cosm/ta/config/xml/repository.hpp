/**
 * \file repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/xml/xml_config_repository.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class repository
 * \ingroup ta config xml
 *
 * \brief Collection of all parameter parsers and parse results needed by
 * task allocation algorithms
 */
class repository : public virtual rconfig::xml::xml_config_repository {
 public:
  repository(void) RCPPSW_COLD;
};

NS_END(xml, config, ta, cosm);
