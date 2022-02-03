/**
 * \file block_cluster_vector.hpp
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

#ifndef INCLUDE_COSM_FORAGING_DS_BLOCK_CLUSTER_VECTOR_HPP_
#define INCLUDE_COSM_FORAGING_DS_BLOCK_CLUSTER_VECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>

#include "rcppsw/er/stringizable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

namespace cosm::foraging::repr {
class block_cluster;
} // namespace repr

NS_START(cosm, foraging, ds);

using block3D_cluster_vectorro_type = const cfrepr::block_cluster*;
using block3D_cluster_vectorno_type = cfrepr::block_cluster*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \class block3D_cluster_vectorro
 * \ingroup foraging ds
 *
 * \brief Specialization of \ref std::vector for block clusters indicating the
 * block clusters are NOT owned by this class and have read only access.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_cluster_vectorro : public rpdecorator::decorator<std::vector<block3D_cluster_vectorro_type>>,
                                 rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(front);
  RCPPSW_DECORATE_DECLDEF(size);

  std::string to_str(void) const override;
};

/**
 * \class block3D_cluster_vectorno
 * \ingroup foraging ds
 *
 * \brief Specialization of \ref std::vector for block clusters indicating that
 * the block clusters are NOT owned by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_cluster_vectorno : public rpdecorator::decorator<std::vector<block3D_cluster_vectorno_type>>,
                                 public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECL(value_type);

  RCPPSW_DECORATE_CT();

  RCPPSW_DECORATE_DECLDEF(operator[]);
  RCPPSW_DECORATE_DECLDEF(operator[], const);
  RCPPSW_DECORATE_DECLDEF(size, const);
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(erase);
  RCPPSW_DECORATE_DECLDEF(clear);
  RCPPSW_DECORATE_DECLDEF(insert);
  RCPPSW_DECORATE_DECLDEF(front);
  RCPPSW_DECORATE_DECLDEF(size);

  std::string to_str(void) const override;
};

NS_END(ds, foraging, cosm);

#endif /* INCLUDE_COSM_DS_BLOCK_CLUSTER_VECTOR_HPP_ */
