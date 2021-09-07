/**
 * \file loctree.cpp
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
#include "cosm/arena/ds/loctree.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, ds);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <typename TEntity>
void loctree::do_update(const TEntity* ent) {
  remove(ent);
  decoratee().insert(ent->id(),
                     rds::make_rtree_box(ent->ranchor2D(),
                                         ent->ranchor2D() + ent->rdim2D()));
} /* do_update() */

size_t loctree::remove(const crepr::base_entity* ent) {
  return decoratee().remove(ent->id());
} /* remove() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void loctree::do_update(const crepr::unicell_entity2D*);
template void loctree::do_update(const crepr::unicell_entity3D*);

NS_END(ds, arena, cosm);
