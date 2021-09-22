/**
 * \file block3D_vector.hpp
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

#ifndef INCLUDE_COSM_DS_BLOCK3D_VECTOR_HPP_
#define INCLUDE_COSM_DS_BLOCK3D_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <vector>

#include "rcppsw/er/stringizable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_vectoro_type = std::unique_ptr<crepr::base_block3D>;
using block3D_vectorno_type = crepr::base_block3D*;
using block3D_vectorro_type = const crepr::base_block3D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class block3D_vectoro
 * \ingroup ds
 *
 * \brief Specialization of \ref std::vector indicating the blocks are OWNED by
 * this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_vectoro : public rpdecorator::decorator<std::vector<block3D_vectoro_type>>,
                        public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);
  RCPPSW_DECORATE_DECL(iterator);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty, const);

  std::string to_str(void) const override;
};

/*
 * \brief Specialization of \ref std::vector indicating the blocks are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_vectorno : public rpdecorator::decorator<std::vector<block3D_vectorno_type>>,
                         public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);
  RCPPSW_DECORATE_DECL(iterator);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty, const);

  std::string to_str(void) const override;
};

/*
 * \brief Specialization of \ref std::vector indicating the blocks are NOT owned
 * by this class and have read only access.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_vectorro : public rpdecorator::decorator<std::vector<block3D_vectorro_type>>,
                         public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);
  RCPPSW_DECORATE_DECL(iterator);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(empty, const);

  std::string to_str(void) const override;
};

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_DS_BLOCK3D_VECTOR_HPP_ */
