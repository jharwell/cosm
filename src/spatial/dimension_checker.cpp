/**
 * \file dimension_checker.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/dimension_checker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d
dimension_checker::even_multiple(const rtypes::discretize_ratio& res,
                                 const rmath::vector2d& to_check) {
  return { even_multiple(res, rtypes::spatial_dist(to_check.x())).v(),
           even_multiple(res, rtypes::spatial_dist(to_check.y())).v() };
} /* even_multiple() */

rmath::vector2d dimension_checker::odd_dsize(const rtypes::discretize_ratio& res,
                                             const rmath::vector2d& to_check) {
  auto checked = to_check;
  auto ddims = rmath::dvec2zvec(to_check, res.v());

  /*
   * We arbitrarily choose to subtract, rather than add one unit of size if
   * needed--Princple Of Least Surprise at work again!
   */
  if (RCPPSW_IS_EVEN(ddims.x())) {
    checked.x(checked.x() - res.v());
  }
  if (RCPPSW_IS_EVEN(ddims.y())) {
    checked.y(checked.y() - res.v());
  }
  return checked;
} /* even_multiple() */

rtypes::spatial_dist
dimension_checker::even_multiple(const rtypes::discretize_ratio& res,
                                 const rtypes::spatial_dist& to_check) {
  double remainder = std::remainder(to_check.v(), res.v());
  rtypes::spatial_dist checked = to_check;

  if (remainder >= std::numeric_limits<double>::epsilon()) {
    checked -= remainder;
  }
  return checked;
} /* even_multiple() */

rtypes::spatial_dist
dimension_checker::odd_dsize(const rtypes::discretize_ratio& res,
                             const rtypes::spatial_dist& to_check) {
  auto checked = to_check;

  auto rdims = rmath::vector2d(to_check.v(), to_check.v());
  auto ddims = rmath::dvec2zvec(rdims, res.v());

  if (RCPPSW_IS_EVEN(ddims.x()) || RCPPSW_IS_EVEN(ddims.y())) {
    checked -= res.v();
  }
  return checked;
} /* odd_dsize() */

NS_END(spatial, cosm);
