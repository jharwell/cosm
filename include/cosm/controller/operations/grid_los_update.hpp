/**
 * \file grid_los_update.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
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
 * \class grid_los_builder
 * \ingroup controller operations detail
 *
 * \brief Compute the line of sight for a given robot as it moves within a 2D/3D
 * grid.
 */
class grid_los_builder {
 public:
  grid_los_builder(void) = default;

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
  grid_los_builder(const grid_los_builder&) = delete;
  grid_los_builder& operator=(const grid_los_builder&) = delete;
  grid_los_builder(grid_los_builder&&) = delete;
  grid_los_builder& operator=(grid_los_builder&&) = delete;
};

/*******************************************************************************
 * Free Functions
 ******************************************************************************/


NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct grid_los_update
 * \ingroup controller operations
 *
 * \brief Functor to update robot LOS each timestep as it moves through a 2D/3D
 * grid.
 */
template <typename TController,
          typename TSrcGrid,
          typename TLOS>
class grid_los_update final
    : public rer::client<grid_los_update<TController,
                                    TSrcGrid,
                                    TLOS>> {
 public:
  explicit grid_los_update(TSrcGrid* const grid)
      : ER_CLIENT_INIT("cosm.controller.operations.grid_los_update"),
        mc_grid(grid) {}

  /*
   * \todo Ideally these would be deleted, but emplace() does not seem to do
   * what I think it does (i.e. construct an object in place without a need for
   * a copy constructor), so it is defaulted instead.
   */
  grid_los_update(const grid_los_update&) = default;
  grid_los_update& operator=(const grid_los_update&) = delete;

  void operator()(TController* const controller) const {
    ER_ASSERT(rmath::is_multiple_of(controller->los_dim(),
                                    mc_grid->resolution().v()),
              "LOS dimension (%f) not an even multiple of grid resolution (%f)",
              controller->los_dim(),
              mc_grid->resolution().v());

    auto los_grid_size = static_cast<size_t>(
        std::round(controller->los_dim() / mc_grid->resolution().v()));
    grid_los_set(controller, los_grid_size);
  }

 private:
  /**
   * \brief Set the LOS of a robot as it moves within a 3D grid.
   *
   * \todo This should eventually be replaced by a calculation of a robot's LOS
   * by the robot, probably using on-board cameras.
   */
  template <typename U = TSrcGrid,
            RCPPSW_SFINAE_DECLDEF(std::is_same<U,
                                  rds::grid3D_overlay<cds::cell3D>>::value)>
  void grid_los_set(TController* const controller, size_t los_grid_units) const {
    controller->perception()->los(make_los3D(controller, los_grid_units));
  }

  /**
   * \brief Set the LOS of a robot as it moves within a 2D grid.
   *
   * \todo This should eventually be replaced by a calculation of a robot's LOS
   * by the robot, probably using on-board cameras.
   */
  template <typename U = TSrcGrid,
            RCPPSW_SFINAE_DECLDEF(std::is_same<U,
                                  rds::grid2D_overlay<cds::cell2D>>::value)>
  void grid_los_set(TController* const controller, size_t los_grid_units) const {
    controller->perception()->los(make_los2D(controller, los_grid_units));
  }

  std::unique_ptr<TLOS> make_los2D(const TController* controller,
                                   size_t los_grid_units) const {
    auto robot_dpos = rmath::dvec2zvec(controller->rpos2D() - mc_grid->originr(),
                                       mc_grid->resolution().v());
    return std::make_unique<TLOS>(controller->entity_id(),
                                  mc_grid->subcircle(robot_dpos,
                                                     los_grid_units),
                                  mc_grid->resolution());
  }

  std::unique_ptr<TLOS> make_los3D(const TController* controller,
                                   size_t los_grid_units) const {
    auto robot_dpos = rmath::dvec2zvec(controller->rpos3D() - mc_grid->originr(),
                                       mc_grid->resolution().v());
    return std::make_unique<TLOS>(controller->entity_id(),
                                  mc_grid->subcircle(robot_dpos,
                                                     los_grid_units),
                                  mc_grid->resolution());
  }
  /* clang-format off */
  const TSrcGrid* const mc_grid;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

