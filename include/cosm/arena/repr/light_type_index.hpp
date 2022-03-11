/**
 * \file light_type_index.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include <map>
#include <string>

#include "rcppsw/utils/color.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, repr);

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

NS_END(repr, arena, cosm);
