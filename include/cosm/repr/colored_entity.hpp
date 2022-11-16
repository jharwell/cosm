/**
 * \file colored_entity.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/utils/color.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class colored_entity
 * \ingroup repr
 *
 * \brief A mixin class representing cell entities that have a static color.
 */
class colored_entity {
 public:
  /**
   * \brief Initialize an entity with a color.
   */
  explicit colored_entity(const rcppsw::utils::color& color) : m_color(color) {}

  colored_entity(const colored_entity& other) = default;
  colored_entity& operator=(const colored_entity& other) = default;

  virtual ~colored_entity(void) = default;

  const rcppsw::utils::color& color(void) const { return m_color; }

 private:
  /* clang-format off */
  rcppsw::utils::color m_color;
  /* clang-format on */
};

NS_END(repr, cosm);
