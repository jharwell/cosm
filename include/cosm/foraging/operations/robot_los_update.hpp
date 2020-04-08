/**
 * \file robot_los_update.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_OPERATIONS_ROBOT_LOS_UPDATE_HPP_
#define INCLUDE_COSM_FORAGING_OPERATIONS_ROBOT_LOS_UPDATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <limits>

#include "rcppsw/er/client.hpp"

#include "cosm/foraging/utils/utils.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_los_update
 * \ingroup foraging operations
 *
 * \brief Functor to update robot LOS each timestep.
 */
template <typename TControllerType, typename TArenaMapType>
class robot_los_update final
    : public boost::static_visitor<void>,
      public rer::client<robot_los_update<TControllerType, TArenaMapType>> {
 public:
  explicit robot_los_update(TArenaMapType* const map)
      : ER_CLIENT_INIT("cosm.support.robot_los_update"), m_map(map) {}

  /*
   * \todo Ideally these would be deleted, but emplace() does not seem to do
   * what I think it does (i.e. construct an object in place without a need for
   * a copy constructor), so it is defaulted instead.
   */
  robot_los_update(const robot_los_update&) = default;
  robot_los_update& operator=(const robot_los_update&) = delete;

  void operator()(TControllerType* const c) const {
    double mod = std::fmod(c->los_dim(), m_map->grid_resolution().v());

    /*
     * Some values of LOS dim and/or grid resolution might not be able to be
     * represented exactly, so we can't just assert that the mod result =
     * 0.0. Instead, we verify that IF the mod result is > 0.0 that it is also
     * VERY close to the grid resolution.
     */
    if (mod >= std::numeric_limits<double>::epsilon()) {
      ER_ASSERT(
          std::fabs(m_map->grid_resolution().v() - mod) <=
              std::numeric_limits<double>::epsilon(),
          "LOS dimension (%f) not an even multiple of grid resolution (%f)",
          c->los_dim(),
          m_map->grid_resolution().v());
    }
    uint los_grid_size = static_cast<uint>(
        std::round(c->los_dim() / m_map->grid_resolution().v()));
    utils::set_robot_los(c, los_grid_size, *m_map);
  }

 private:
  /* clang-format off */
  TArenaMapType* const m_map;
  /* clang-format on */
};

NS_END(operations, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_OPERATIONS_ROBOT_LOS_UPDATE_HPP_ */
