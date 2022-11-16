/**
 * \file block3D_vector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
class sim_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds);

using block3D_vectoro_type = std::unique_ptr<crepr::sim_block3D>;
using block3D_vectorno_type = crepr::sim_block3D*;
using block3D_vectorro_type = const crepr::sim_block3D*;

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
class block3D_vectoro
    : public rpdecorator::decorator<std::vector<block3D_vectoro_type>>,
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
class block3D_vectorno
    : public rpdecorator::decorator<std::vector<block3D_vectorno_type>>,
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

  std::string to_str(void) const override final;
};

/*
 * \brief Specialization of \ref std::vector indicating the blocks are NOT owned
 * by this class and have read only access.
 *
 * Has a \ref to_str() method for more convenient debugging.
 */
class block3D_vectorro
    : public rpdecorator::decorator<std::vector<block3D_vectorro_type>>,
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

  std::string to_str(void) const override final;
};

NS_END(ds, cosm);
