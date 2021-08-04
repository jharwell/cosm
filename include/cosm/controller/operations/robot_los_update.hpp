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

#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_ROBOT_LOS_UPDATE_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_ROBOT_LOS_UPDATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant/static_visitor.hpp>
#include <limits>
#include <cmath>
#include <utility>
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/ds/grid2D_overlay.hpp"
#include "rcppsw/ds/grid3D_overlay.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_los_builder
 * \ingroup controller operations detail
 *
 * \brief Compute the line of sight for a given robot as it moves within a 2D/3D
 * grid.
 */
class robot_los_builder {
 public:
  robot_los_builder(void) = default;

  template <typename TGrid, typename TLOS>
  std::unique_ptr<TLOS> operator()(
      const TGrid* const grid,
      const typename TLOS::field_coord_rtype& robot_rpos,
      const rtypes::type_uuid& robot_id,
      size_t los_grid_size) {
    auto robot_dpos = rmath::dvec2zvec(robot_rpos - grid->originr(),
                                       grid->resolution().v());
    return std::make_unique<TLOS>(robot_id,
                                  grid->subcircle(robot_dpos,
                                                  los_grid_size),
                                  grid->resolution());
  } /* robot_los_compute */

  /* Not move/copy constructable/assignable by default */
  robot_los_builder(const robot_los_builder&) = delete;
  robot_los_builder& operator=(const robot_los_builder&) = delete;
  robot_los_builder(robot_los_builder&&) = delete;
  robot_los_builder& operator=(robot_los_builder&&) = delete;
};

/*******************************************************************************
 * Free Functions
 ******************************************************************************/

/**
 * \brief Set the LOS of a robot as it moves within a 3D grid.
 *
 * \todo This should eventually be replaced by a calculation of a robot's LOS by
 * the robot, probably using on-board cameras.
 */
template <typename TController, typename TLOS, typename TGrid,
          RCPPSW_SFINAE_DECLDEF(std::is_same<TGrid,
                                rds::grid3D_overlay<cds::cell3D>>::value)>
void robot_los_set(TController* const controller,
                   const TGrid* const grid,
                   size_t los_grid_units) {
  auto builder = robot_los_builder();
  auto los = builder.operator()<TGrid, TLOS>(grid,
                                             controller->rpos3D(),
                                             controller->entity_id(),
                                             los_grid_units);
  controller->perception()->los(std::move(los));
}

/**
 * \brief Set the LOS of a robot as it moves within a 2D grid.
 *
 * \todo This should eventually be replaced by a calculation of a robot's LOS by
 * the robot, probably using on-board cameras.
 */
template <typename TController, typename TLOS, typename TGrid,
          RCPPSW_SFINAE_DECLDEF(std::is_same<TGrid,
                                rds::grid2D_overlay<cds::cell2D>>::value)>
void robot_los_set(TController* const controller,
                   const TGrid* const grid,
                   size_t los_grid_units) {
  auto builder = robot_los_builder();
  auto los = builder.operator()<TGrid, TLOS>(grid,
                                             controller->rpos2D(),
                                             controller->entity_id(),
                                             los_grid_units);
  controller->perception()->los(std::move(los));
}

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct robot_los_update
 * \ingroup controller operations
 *
 * \brief Functor to update robot LOS each timestep.
 */
template <typename TController,
          typename TSrcGrid,
          typename TLOS>
class robot_los_update final
    : public rer::client<robot_los_update<TController,
                                          TSrcGrid,
                                          TLOS>>,
      private boost::static_visitor<void> {
 public:
  explicit robot_los_update(TSrcGrid* const grid)
      : ER_CLIENT_INIT("cosm.support.robot_los_update"), mc_grid(grid) {}

  /*
   * \todo Ideally these would be deleted, but emplace() does not seem to do
   * what I think it does (i.e. construct an object in place without a need for
   * a copy constructor), so it is defaulted instead.
   */
  robot_los_update(const robot_los_update&) = default;
  robot_los_update& operator=(const robot_los_update&) = delete;

  void operator()(TController* const controller) const {
    double mod = std::fmod(controller->los_dim(), mc_grid->resolution().v());

    /*
     * Some values of LOS dim and/or grid resolution might not be able to be
     * represented exactly, so we can't just assert that the mod result =
     * 0.0. Instead, we verify that IF the mod result is > 0.0 that it is also
     * VERY close to the grid resolution.
     */
    if (mod >= std::numeric_limits<double>::epsilon()) {
      ER_ASSERT(
          std::fabs(mc_grid->resolution().v() - mod) <=
              std::numeric_limits<double>::epsilon(),
          "LOS dimension (%f) not an even multiple of grid resolution (%f)",
          controller->los_dim(),
          mc_grid->resolution().v());
    }
    auto los_grid_size = static_cast<size_t>(
        std::round(controller->los_dim() / mc_grid->resolution().v()));
    detail::robot_los_set<TController, TLOS>(controller,
                                             mc_grid,
                                             los_grid_size);
  }

 private:
  /* clang-format off */
  TSrcGrid* const mc_grid;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_ROBOT_LOS_UPDATE_HPP_ */
