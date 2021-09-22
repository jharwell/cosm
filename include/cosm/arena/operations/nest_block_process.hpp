 /**
 * \file nest_block_process.hpp
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

#ifndef INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_PROCESS_HPP_
#define INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_PROCESS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/er/client.hpp"
#include "rcppsw/patterns/visitor/visitor.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/mpl/typelist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena {
class base_arena_map;
class caching_arena_map;
} /* namespace cosm::arena */

namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_block_process
 * \ingroup arena operations detail
 *
 * \brief Fired whenever a robot brings a block to the \ref crepr::nest, and
 * meets the conditions for processing, "processing" having an application
 * dependent meaning, hence the generic name of this class.
 *
 * -
 */
class nest_block_process : public rer::client<nest_block_process> {
 private:
  struct visit_typelist_impl {
    using value = rmpl::typelist<base_arena_map,
                                 caching_arena_map,
                                 crepr::base_block3D>;
  };

 public:
  using visit_typelist = visit_typelist_impl::value;

  /**
   * \brief Initialize a nest block process event.
   *
   * \param arena_block The arena map owned block with the same ID as the block
   *                    the robot released for this processing event.
   * \param t Current timestep.
   */
  nest_block_process(crepr::base_block3D* arena_block,
                     const rtypes::timestep& t);

  ~nest_block_process(void) override = default;

  nest_block_process(const nest_block_process&) = delete;
  nest_block_process& operator=(const nest_block_process&) = delete;

  /**
   * \brief Perform actual nest block process in the arena.
   *
   * Takes \ref caching_arena_map block and grid mutexes to protect block
   * re-distribution and block updates, and releases afterwards. See
   * COSM#594.
   */
  void visit(caching_arena_map& map);
  void visit(base_arena_map& map);

 private:
  void visit(crepr::base_block3D& block);

  template <typename TArenaMap>
  void do_visit(TArenaMap& map);

  /* clang-format off */
  const rtypes::timestep               mc_timestep;
  crepr::base_block3D*                 m_arena_block;
  /* clang-format on */
};

NS_END(detail);

/**
 * \brief We use the precise visitor in order to force compile errors if a call
 * to a visitor is made that involves a visitee that is not in our visit set
 * (i.e. remove the possibility of implicit upcasting performed by the
 * compiler).
 */
using nest_block_process_visitor = rpvisitor::filtered_visitor<detail::nest_block_process>;

NS_END(operations, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_OPERATIONS_NEST_BLOCK_PROCESS_HPP_ */
