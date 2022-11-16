/**
 * \file repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/repository.hpp"

#include "cosm/ta/config/xml/task_alloc_parser.hpp"
#include "cosm/ta/config/xml/task_executive_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
repository::repository(void) {
  parser_register<cta::config::xml::task_alloc_parser,
                  cta::config::task_alloc_config>(
      cta::config::xml::task_alloc_parser::kXMLRoot);
  parser_register<cta::config::xml::task_executive_parser,
                  cta::config::task_executive_config>(
      cta::config::xml::task_executive_parser::kXMLRoot);
}

NS_END(xml, config, ta, cosm);
