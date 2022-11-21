/**
 * \file block_cluster_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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

namespace cosm::foraging::ds {

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

} /* namespace cosm::foraging::ds */
