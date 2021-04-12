/**
 * \file embodied_entity.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_REPR_EMBODIED_ENTITY_HPP_
#define INCLUDE_COSM_REPR_EMBODIED_ENTITY_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class embodied_entity
 * \ingroup repr
 *
 * \brief An entity that has a 3D/physical embodiment in the arena.
 */
template<typename TEmbodimentType>
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

  const TEmbodimentType* embodiment(void) const {
    return m_embodiment.get();
  }
  void embodiment(std::unique_ptr<TEmbodimentType> e) {
    m_embodiment = std::move(e);
  }

 private:
  /* clang-format off */
  std::unique_ptr<TEmbodimentType> m_embodiment{};
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_EMBODIED_ENTITY_HPP_ */
