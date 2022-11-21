/**
 * \file embodied_entity.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class embodied_entity
 * \ingroup repr
 *
 * \brief An entity that has a 3D/physical embodiment in the arena.
 */
template <typename TEmbodimentType>
class embodied_entity {
 public:
  explicit embodied_entity(std::unique_ptr<TEmbodimentType> e)
      : m_embodiment(std::move(e)) {}

  embodied_entity(void) = default;
  virtual ~embodied_entity(void) = default;

  /* Not move/copy constructable/assignable by default */
  embodied_entity(const embodied_entity&) = delete;
  embodied_entity& operator=(const embodied_entity&) = delete;
  embodied_entity(embodied_entity&&) = delete;
  embodied_entity& operator=(embodied_entity&&) = delete;

  const TEmbodimentType* embodiment(void) const { return m_embodiment.get(); }
  void embodiment(std::unique_ptr<TEmbodimentType> e) {
    m_embodiment = std::move(e);
  }

 private:
  /* clang-format off */
  std::unique_ptr<TEmbodimentType> m_embodiment{};
  /* clang-format on */
};

} /* namespace cosm::repr */
