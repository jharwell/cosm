/**
 * \file penalty_id_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TV_PENALTY_ID_CALCULATOR_HPP_
#define INCLUDE_COSM_TV_PENALTY_ID_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {
class arena_map;
} /* namespace cosm::arena */

NS_START(cosm, foraging, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class penalty_id_calculator
 * \ingroup foraging tv
 *
 * \brief Calculates the ID for a \ref temporal_penalty under some general
 * purpose conditions. Projects built on COSM will undoubtedly have to define
 * their own additional ID calculation functions/classes to extend the
 * functionality present here.
 */
class penalty_id_calculator : public rer::client<penalty_id_calculator> {
 public:
  penalty_id_calculator(void)
      : ER_CLIENT_INIT("cosm.tv.penalty_id_calculator") {}

  /* Not copy constructable/assignable by default */
  penalty_id_calculator(const penalty_id_calculator&) = delete;
  const penalty_id_calculator& operator=(const penalty_id_calculator&) = delete;

  /**
   * \brief Compute the ID for the penalty if the operation involves dropping a
   * block in the \ref crepr::nest.
   *
   * \param block The block the robot is currently carrying.
   */
  rtypes::type_uuid from_nest_drop(const crepr::base_block2D* block) const RCSW_PURE;

  /**
   * \brief Compute the ID for the penalty if the operation involves picking up
   * a free block in the arena.
   *
   * \param loc The robots current location.
   * \param acq_id The ID of the thing (probably a block) that the robot THINKS
   *               it has acquired.
   * \param map The \ref carena::arena_map.
   */
  rtypes::type_uuid from_free_pickup(const rmath::vector2d& loc,
                                     const rtypes::type_uuid& acq_id,
                                     const carena::arena_map* map) const RCSW_PURE;
};

NS_END(tv, foraging, cosm);

#endif /* INCLUDE_COSM_TV_PENALTY_ID_CALCULATOR_HPP_ */
