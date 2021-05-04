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

#ifndef INCLUDE_COSM_ARENA_DS_LOCTREE_HPP_
#define INCLUDE_COSM_ARENA_DS_LOCTREE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/rtree2D.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/repr/unicell_entity2D.hpp"
#include "cosm/repr/unicell_entity3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, ds, detail);

using rtree_type = rds::rtree2D<double,
                                rtypes::type_uuid,
                                16>;  /* Max # elements per node */
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
 * rtypes::type_uuid does not guarantee uniqueness_across types (duh).
 */
class loctree final : public rpdecorator::decorator<detail::rtree_type> {
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

  RCPPSW_DECORATE_DECLDEF(query, const);
  RCPPSW_DECORATE_DECLDEF(begin, const);
  RCPPSW_DECORATE_DECLDEF(end, const);
  RCPPSW_DECORATE_DECLDEF(size, const);

 private:
  template<typename TEntity>
  void do_update(const TEntity* ent);

  /* clang-format off */
  /* clang-format on */
};

NS_END(ds, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_DS_LOCTREE_HPP_ */
