/**
 * \file repository.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
