/**
 * \file dimension_checker.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial {

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
  static rspatial::euclidean_dist even_multiple(const rtypes::discretize_ratio& res,
                                            const rspatial::euclidean_dist& to_check);

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
  static rspatial::euclidean_dist odd_dsize(const rtypes::discretize_ratio& res,
                                        const rspatial::euclidean_dist& to_check);
};

} /* namespace cosm::spatial */
