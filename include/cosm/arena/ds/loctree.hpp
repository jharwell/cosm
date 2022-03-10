/**
 * \file loctree.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/rtree.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/repr/unicell_entity2D.hpp"
#include "cosm/repr/unicell_entity3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, ds, detail);

using rtree_spec_type = rds::rtree_spec<rmath::vector2d,
                                        rds::rtree_box<rmath::vector2d>,
                                        rtypes::type_uuid>;
NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class loctree
 * \ingroup arena ds
 *
 * \brief Store an Rtree representation of a type of entities in the arena for
 * fast querying.
 *
 * \note You can't mix multiple types of entities, and the \ref
 * rtypes::type_uuid does not guarantee uniqueness across types (duh).
 */
class loctree final : public rpdecorator::decorator<rds::rtree<detail::rtree_spec_type>> {

 private:
  template<typename TEntity>
  void do_update(const TEntity* ent);

 public:
  loctree(void) = default;

  /* Not move/copy constructable/assignable by default */
  loctree(const loctree&) = delete;
  const loctree& operator=(const loctree&) = delete;
  loctree(loctree&&) = delete;
  loctree& operator=(loctree&&) = delete;

  /*
   * \brief Update entity location query tree by removing the old tree entry if
   * it exists and add the new one.
   */
  void update(const crepr::unicell_entity2D* ent) { do_update(ent); }

  /*
   * \brief Update entity location query tree by removing the old tree entry if
   * it exists and add the new one.
   */
  void update(const crepr::unicell_entity3D* ent) { do_update(ent); }

  size_t remove(const crepr::base_entity* ent);

  RCPPSW_DECORATE_DECLDEF(intersections, const);
  RCPPSW_DECORATE_DECLDEF(contains, const);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(size, const);
};

NS_END(ds, arena, cosm);
