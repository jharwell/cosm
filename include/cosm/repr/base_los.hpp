/**
 * \file base_los.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_LOS_HPP_
#define INCLUDE_COSM_REPR_BASE_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/base_grid2D.hpp"
#include "rcppsw/ds/base_grid3D.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ds {
class cell2D;
class cell3D;
} /* namespace cosm::ds */

NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_los
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight.. The robot is only able
 * to update its internal state based on the information present in the
 * per-timestep updates to this object.
 *
 * The LOS for a robot is always square UNLESS the robot is near the edge of the
 * source field, and a square grid would result in out-of-bounds array
 * accesses. In that case, a truncated LOS is created.
 *
 * All coordinates within a LOS are relative to the LOS itself (not its location
 * within the arena). The origin is in the lower left corner of the LOS.
 */
template <typename TCell>
class base_los : public rer::client<base_los<TCell>> {
 public:
  using grid_view =
      typename std::conditional<std::is_same<cds::cell2D, TCell>::value,
                                typename rds::base_grid2D<TCell>::grid_view,
                                typename rds::base_grid3D<TCell>::grid_view>::type;

  using const_grid_view = typename std::conditional<
      std::is_same<cds::cell2D, TCell>::value,
      typename rds::base_grid2D<TCell>::const_grid_view,
      typename rds::base_grid3D<TCell>::const_grid_view>::type;

  using field_coord_type =
      typename std::conditional<std::is_same<cds::cell2D, TCell>::value,
                                rmath::vector2z,
                                rmath::vector3z>::type;
  using los_coord_type = rmath::vector2z;

  explicit base_los(const const_grid_view& c_view)
      : ER_CLIENT_INIT("cosm.repr.base_los"), mc_view(c_view) {}

  /**
   * \brief Get the cell associated with a particular grid location within the
   * LOS. Asserts that both coordinates are within the bounds of the grid
   * underlying the LOS.
   *
   * \param c The RELATIVE coord within the LOS.
   *
   * \return A reference to the cell.
   */
  virtual const TCell& access(const los_coord_type& c) const = 0;

  virtual field_coord_type abs_ll(void) const = 0;
  virtual field_coord_type abs_ul(void) const = 0;
  virtual field_coord_type abs_lr(void) const = 0;
  virtual field_coord_type abs_ur(void) const = 0;

  /**
   * \brief Determine if the coordinates from the parent field are contained in
   * the LOS.
   */
  virtual bool contains_abs(const field_coord_type& coord) const = 0;

  /**
   * \brief Determine if the LOS RELATIVE coordinates are contained in the LOS.
   */
  virtual bool contains_rel(const los_coord_type& coord) const = 0;

  /**
   * \brief Get the size of the X dimension for a LOS.
   *
   * \return The X dimension.
   */
  typename grid_view::size_type xsize(void) const { return mc_view.shape()[0]; }

  /**
   * \brief Get the size of the Y dimension for a LOS.
   *
   * \return The Y dimension.
   */
  typename grid_view::size_type ysize(void) const { return mc_view.shape()[1]; }

 protected:
  const const_grid_view& view(void) const { return mc_view; }

 private:
  /* clang-format off */
  const const_grid_view mc_view;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_LOS_HPP_ */
