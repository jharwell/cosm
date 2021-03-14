/**
 * \file block3D_list.hpp
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

#ifndef INCLUDE_COSM_DS_BLOCK3D_LIST_HPP_
#define INCLUDE_COSM_DS_BLOCK3D_LIST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <list>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_listno_type = crepr::base_block3D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/*
 * \brief Specialization of \ref std::list indicating the blocks are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_listno : public std::list<block3D_listno_type> {
 public:
  using std::list<block3D_listno_type>::list;
  using value_type = std::list<block3D_listno_type>::value_type;

  /**
   * \brief Get a string representation of the list contents.
   */
  std::string to_str(void) const;
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_DS_BLOCK3D_LIST_HPP_ */
