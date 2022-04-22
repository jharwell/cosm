/**
 * \file name_spec.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, metrics, specs);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class name_spec
 * \ingroup metrics specs
 *
 * \brief Represents the specification of metrics to be collected. It is the
 * interface between the input name of the metrics in an XML input file, and the
 * (scoped) runtime name of the metrics when they are collected from objects.
 */
class name_spec {
 public:
  name_spec(const std::string& xml, const std::string& scoped);

  /**
   * \brief If \p id is passed, then the string \p '__UUID__' will be replaced
   * by the string representing the passed id in the return XML name.
   */
  std::string xml(const rtypes::type_uuid& id = rtypes::constants::kNoUUID) const;

  /**
   * \brief If \p id is passed, then the string \p '__UUID__' will be replaced
   * by the string representing the passed id in the return scoped name.
   */
  std::string
  scoped(const rtypes::type_uuid& id = rtypes::constants::kNoUUID) const;

 private:
  /* clang-format off */
  std::string       m_xml;
  std::string       m_scoped;
  /* clang-format on */
};

NS_END(specs, metrics, cosm);
