/**
 * \file governed_diff_drive.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin2D/governed_diff_drive.hpp"

#include <numeric>

#include "cosm/tv/switchable_tv_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
double governed_diff_drive::active_throttle(void) const {
  return std::accumulate(std::begin(m_generators),
                         std::end(m_generators),
                         0.0,
                         [&](double sum, const auto* generator){
                           return sum + generator->active_tv();
                         });
}

double governed_diff_drive::applied_throttle(void) const {
  return std::accumulate(std::begin(m_generators),
                         std::end(m_generators),
                         0.0,
                         [&](double sum, const auto* generator){
                           return sum + generator->applied_tv();
                         });
}
NS_END(kin2d, cosm);
