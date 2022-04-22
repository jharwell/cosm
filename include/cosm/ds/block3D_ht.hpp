/**
 * \file block3D_ht.hpp
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
#include <memory>
#include <string>
#include <unordered_map>

#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_ht_key_type = rtypes::type_uuid;

using block3D_hto_value_type = std::shared_ptr<crepr::sim_block3D>;
using block3D_htno_value_type = crepr::sim_block3D*;
using block3D_htro_value_type = const crepr::sim_block3D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
NS_START(detail);

struct hash_function {
  decltype(std::declval<rtypes::type_uuid>().v())
  operator()(const rtypes::type_uuid& key) const {
    return key.v();
  }
};

NS_END(detail);

/**
 * \class block3D_hto
 * \ingroup ds
 *
 * \brief Specialization of \ref std::ht indicating the blocks are OWNED by
 * this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_hto
    : public rpdecorator::decorator<std::unordered_map<block3D_ht_key_type,
                                                       block3D_hto_value_type,
                                                       detail::hash_function>>,
      public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(iterator);
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(find);
  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(size, const);

  std::string to_str(void) const override;
};

/*
 * \brief Specialization of \ref std::unordered_map indicating the blocks are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_htno
    : public rpdecorator::decorator<std::unordered_map<block3D_ht_key_type,
                                                       block3D_htno_value_type,
                                                       detail::hash_function>>,
      public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(iterator);
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(find);
  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(size, const);

  std::string to_str(void) const override;
};

/*
 * \brief Specialization of \ref std::unordered_map indicating the blocks are NOT owned
 * by this class and have read only access.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_htro
    : public rpdecorator::decorator<std::unordered_map<block3D_ht_key_type,
                                                       block3D_htro_value_type,
                                                       detail::hash_function>>,
      public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(iterator);
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(find);
  RCPPSW_DECORATE_DECLDEF(find, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(size, const);

  std::string to_str(void) const override;
};

NS_END(ds, cosm);
