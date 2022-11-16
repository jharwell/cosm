/**
 * \file dimension_checker.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
  return { even_multiple(res, rspatial::euclidean_dist(to_check.x())).v(),
           even_multiple(res, rspatial::euclidean_dist(to_check.y())).v() };
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

rspatial::euclidean_dist
dimension_checker::even_multiple(const rtypes::discretize_ratio& res,
                                 const rspatial::euclidean_dist& to_check) {
  double remainder = std::remainder(to_check.v(), res.v());
  rspatial::euclidean_dist checked = to_check;

  if (remainder >= std::numeric_limits<double>::epsilon()) {
    checked -= remainder;
  }
  return checked;
} /* even_multiple() */

rspatial::euclidean_dist
dimension_checker::odd_dsize(const rtypes::discretize_ratio& res,
                             const rspatial::euclidean_dist& to_check) {
  /*
   * Add a TINY bit of extra padding here to fix floating point representation
   * errors.
   */
  auto checked = to_check + rmath::kDOUBLE_EPSILON;
  auto rdims = rmath::vector2d(checked.v(), checked.v());
  auto ddims = rmath::dvec2zvec(rdims, res.v());

  if (RCPPSW_IS_EVEN(ddims.x()) || RCPPSW_IS_EVEN(ddims.y())) {
    auto checked_sub = checked;
    checked_sub -= res.v();
    rdims = rmath::vector2d(checked_sub.v(), checked_sub.v());
    ddims = rmath::dvec2zvec(rdims, res.v());
    if (RCPPSW_IS_ODD(ddims.x()) && RCPPSW_IS_ODD(ddims.y())) {
      return checked_sub;
    }
    auto checked_add = checked;
    checked_add += res.v();
    rdims = rmath::vector2d(checked_add.v(), checked_add.v());
    ddims = rmath::dvec2zvec(rdims, res.v());
    if (RCPPSW_IS_ODD(ddims.x()) && RCPPSW_IS_ODD(ddims.y())) {
      return checked_add;
    }
  }
  return checked;
} /* odd_dsize() */

NS_END(spatial, cosm);
