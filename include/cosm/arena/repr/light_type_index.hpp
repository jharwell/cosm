/**
 * \file light_type_index.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>

#include "rcppsw/utils/color.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class light_type_index
 * \ingroup arena repr
 *
 * \brief Index mapping an entity type to the color of the light that should be
 * associated with it, so that what things should have lights of what color is
 * not hardcoded in multiple places in the code, and is instead centralized
 * here. For usage by both robots and loop functions.
 *
 * Currently maps:
 *
 * - \ref repr::nest
 * - \ref repr::arena_cache
 */

class light_type_index {
 public:
  static inline const std::string kNest = "nest";
  static inline const std::string kCache = "cache";

  light_type_index(void);

  const rutils::color& operator[](const std::string& key) {
    return m_index[key];
  }

 private:
  /* clang-format off */
  std::map<std::string, rutils::color> m_index;
  /* clang-format on */
};

} /* namespace cosm::arena::repr */
