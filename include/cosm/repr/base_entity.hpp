/**
 * \file base_entity.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/entity_dimensionality.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_entity
 * \ingroup repr
 *
 * \brief A base class from which all entities that can be represented in the
 * arena derive.
 */
class base_entity {
 public:
  base_entity(void) : base_entity{ rtypes::constants::kNoUUID } {}
  explicit base_entity(const rtypes::type_uuid& id) : m_id(id) {}

  base_entity(const base_entity&) = default;
  base_entity& operator=(const base_entity&) = default;

  virtual ~base_entity(void) = default;

  /**
   * \brief Set the ID of the object.
   */
  void id(const rtypes::type_uuid& id) { m_id = id; }

  /**
   * \brief Get the ID of the object.
   */
  const rtypes::type_uuid& id(void) const { return m_id; }

  /**
   * \brief Set if the ID of the entity should be visualized or not if
   * visualizations are enabled.
   */
  void vis_id(bool b) { m_vis_id = b; }

  /**
   * \brief Get the entity ID visualization status.
   */
  bool vis_id(void) const { return m_vis_id; }

 private:
  /* clang-format off */
  bool              m_vis_id{false};
  rtypes::type_uuid m_id{rtypes::constants::kNoUUID};
  /* clang-format on */
};

} /* namespace cosm::repr */
