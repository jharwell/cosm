/**
 * \file block3D_list.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <memory>
#include <string>

#include "rcppsw/er/stringizable.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_listno_type = crepr::sim_block3D*;

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/

/*
 * \brief Specialization of \ref std::list indicating the blocks are NOT owned
 * by this class.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_listno
    : public rpdecorator::decorator<std::list<block3D_listno_type>>,
      public rer::stringizable {
 public:
  RCPPSW_DECORATE_DECLDEF(push_back);
  RCPPSW_DECORATE_DECLDEF(begin);
  RCPPSW_DECORATE_DECLDEF(end);

  std::string to_str(void) const override;

 private:
  /* clang-format off */
  std::list<block3D_listno_type> m_impl;
};

NS_END(ds, cosm);

