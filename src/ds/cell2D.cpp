/**
 * \file cell2D.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/ds/cell2D.hpp"

#include "cosm/arena/repr/base_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D::cell2D(void) { decoratee().init(); }

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::base_block2D* cell2D::block2D(void) const {
  return dynamic_cast<crepr::base_block2D*>(m_entity);
} /* block2D() */

crepr::base_block2D* cell2D::block2D(void) {
  return dynamic_cast<crepr::base_block2D*>(m_entity);
} /* block2D() */

crepr::base_block3D* cell2D::block3D(void) const {
  return dynamic_cast<crepr::base_block3D*>(m_entity);
} /* block3D() */

crepr::base_block3D* cell2D::block3D(void) {
  return dynamic_cast<crepr::base_block3D*>(m_entity);
} /* block3D() */

carepr::base_cache* cell2D::cache(void) const {
  return dynamic_cast<carepr::base_cache*>(m_entity);
} /* cache() */

carepr::base_cache* cell2D::cache(void) {
  return dynamic_cast<carepr::base_cache*>(m_entity);
} /* cache() */

NS_END(ds, cosm);
