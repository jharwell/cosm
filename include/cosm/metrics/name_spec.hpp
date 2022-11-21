/**
 * \file name_spec.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
namespace cosm::metrics::specs {

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

} /* namespace cosm::metrics::specs */
