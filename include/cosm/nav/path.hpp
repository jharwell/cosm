/**
 * \file path.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::nav {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class path3D
 * \ingroup nav
 *
 * \brief A representation of a path an agent can move along in 3D space.
 */
class path3D : public rpdecorator::decorator<std::vector<rmath::vector3d>>,
               public rer::stringizable {
 public:
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

  std::string to_str(void) const override;

 private:
  /* clang-format off */
  std::vector<rmath::vector3d> m_impl{};
  /* clang-format on */
};


} /* namespace cosm::nav */
