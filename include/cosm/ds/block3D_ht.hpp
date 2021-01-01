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

#ifndef INCLUDE_COSM_DS_BLOCK3D_HT_HPP_
#define INCLUDE_COSM_DS_BLOCK3D_HT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <unordered_map>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_ht_key_type = rtypes::type_uuid;

using block3D_hto_value_type = std::shared_ptr<crepr::base_block3D>;
using block3D_htno_value_type = crepr::base_block3D*;
using block3D_htro_value_type = const crepr::base_block3D*;

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
class block3D_hto : public std::unordered_map<block3D_ht_key_type,
                                              block3D_hto_value_type,
                                              detail::hash_function> {
 public:
  using std::unordered_map<block3D_ht_key_type,
                           block3D_hto_value_type,
                           detail::hash_function>::unordered_map;
  using value_type = std::unordered_map<block3D_ht_key_type,
                                        block3D_hto_value_type,
                                        detail::hash_function>::value_type;

  /**
   * \brief Get a string representation of the ht contents.
   */
  std::string to_str(void) const;
};

/*
 * \brief Specialization of \ref std::unordered_map indicating the blocks are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_htno : public std::unordered_map<block3D_ht_key_type,
                                               block3D_htno_value_type,
                                               detail::hash_function> {
 public:
  using std::unordered_map<block3D_ht_key_type,
                           block3D_htno_value_type,
                           detail::hash_function>::unordered_map;
  using value_type = std::unordered_map<block3D_ht_key_type,
                                        block3D_htno_value_type,
                                        detail::hash_function>::value_type;

  /**
   * \brief Get a string representation of the ht contents.
   */
  std::string to_str(void) const;
};

/*
 * \brief Specialization of \ref std::unordered_map indicating the blocks are NOT owned
 * by this class and have read only access.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_htro : public std::unordered_map<block3D_ht_key_type,
                                               block3D_htro_value_type,
                                               detail::hash_function> {
 public:
  using std::unordered_map<block3D_ht_key_type,
                           block3D_htro_value_type,
                           detail::hash_function>::unordered_map;
  using value_type = std::unordered_map<block3D_ht_key_type,
                                        block3D_htro_value_type,
                                        detail::hash_function>::value_type;

  /**
   * \brief Get a string representation of the ht contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_DS_BLOCK3D_HT_HPP_ */
