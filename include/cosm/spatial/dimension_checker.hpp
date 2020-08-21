/**
 * \file dimension_checker.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_DIMENSION_CHECKER_HPP_
#define INCLUDE_COSM_SPATIAL_DIMENSION_CHECKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dimension_checker
 * \ingroup spatial
 *
 * \brief Checker to verify and, if necessary, change the passed dimensions to
 * be even multiples of the arena map resolution. This is necessary for LOTS of
 * different calculations to be correct at runtime, including:
 *
 * - Cache placement
 * - Block placement
 */
class dimension_checker {
 public:
  /**
   * \brief Check the spatial dimensions that a class wants to use to create
   * objects with, and modify it if necessary so that it is an even multiple of
   * the grid size.
   */
  static rmath::vector2d even_multiple(const rtypes::discretize_ratio& res,
                                       const rmath::vector2d& to_check);
  static rtypes::spatial_dist even_multiple(const rtypes::discretize_ratio& res,
                                            const rtypes::spatial_dist& to_check);

  /**
   * \brief Check the dimension that a class wants to use to create objects
   * with, and modify it if necessary so that the created objects have an odd #
   * cells in X,Y, so that they have a uniquely defined discrete center.
   *
   * \note This function does NOT call \ref resolution() before possibly
   * shrinking the size of the dimension.
   */
  static rmath::vector2d odd_dsize(const rtypes::discretize_ratio& res,
                                   const rmath::vector2d& to_check);
  static rtypes::spatial_dist odd_dsize(const rtypes::discretize_ratio& res,
                                        const rtypes::spatial_dist& to_check);
};

NS_END(spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_DIMENSION_CHECKER_HPP_ */
