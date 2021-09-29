/**
 * \file temporal_penalty_handler.cpp
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
#include "cosm/tv/temporal_penalty_handler.hpp"

#include <algorithm>

#include "rcppsw/control/waveform_generator.hpp"

#include "cosm/controller/base_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
temporal_penalty_handler::temporal_penalty_handler(
    const ctv::config::temporal_penalty_config* const config,
    const std::string& name)
    : ER_CLIENT_INIT("cosm.tv.temporal_penalty_handler"),
      mc_unique_finish(config->unique_finish),
      mc_name(name),
      m_waveform(rct::waveform_generator()(config->waveform.type,
                                           &config->waveform)) {}


/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void temporal_penalty_handler::penalty_abort(
    const controller::base_controller& controller) {
  lock_wr(&m_list_mtx);
  auto it = penalty_find(controller, false);
  if (m_penalty_list.end() != it) {
    penalty_remove(*it, false);
  }
  unlock_wr(&m_list_mtx);

  ER_INFO("Abort penalty serving for entity%d", controller.entity_id().v());
  ER_ASSERT(!is_serving_penalty(controller, false),
            "Robot still serving penalty after abort?!");
} /* penalty_abort() */

rtypes::timestep temporal_penalty_handler::penalty_calc(
    const rtypes::timestep& t) const {
  rtypes::timestep penalty(0);

  /* can be NULL if penalty handling is disabled */
  if (nullptr != m_waveform) {
    penalty.set(static_cast<uint>(m_waveform->value(t.v())));
  }
  /*
   * If the penalty for the robot was zero, we still need to make the robot
   * serve a 1 timestep penalty. Not needed for block ops (but doesn't really
   * hurt), but IS needed for cache ops, so that if two robots that enter a
   * cache on the same timestep and will serve 0 duration penalties things are
   * still handled properly. You can't rely on just checking the list in that
   * case, because 0 duration penalties are marked as served and removed from
   * the list the SAME timestep they are added, so the handler incorrectly
   * thinks that there is no conflict.
   */
  return penalty += static_cast<uint>(penalty == 0UL);
} /* penalty_calc() */

rtypes::timestep temporal_penalty_handler::penalty_add(
    const controller::base_controller* controller,
    const rtypes::type_uuid& id,
    const rtypes::timestep& orig_duration,
    const rtypes::timestep& start) {
  /*
   * Note that the uniqueify AND actual list add operations must be covered by
   * the SAME lock-unlock sequence (not two separate sequences) in order for
   * all robots to always obey cache pickup policies. See COSM#625.
   */
  lock_wr(&m_list_mtx);
  auto duration = orig_duration;

  if (mc_unique_finish) {
    duration = penalty_finish_uniqueify(start, orig_duration);
  }

  m_penalty_list.push_back(temporal_penalty(controller, id, duration, start));
  unlock_wr(&m_list_mtx);
  return duration;
} /* penalty_add() */

void temporal_penalty_handler::penalty_remove(const temporal_penalty& victim,
                                              bool lock) {
  maybe_lock_wr(&m_list_mtx, lock);
  m_penalty_list.remove(victim);
  maybe_unlock_wr(&m_list_mtx, lock);
} /* penalty_remove() */

temporal_penalty temporal_penalty_handler::penalty_next(void) const {
  lock_rd(&m_list_mtx);
  auto ret = m_penalty_list.front();
  unlock_rd(&m_list_mtx);
  return ret;
} /* penalty_next() */

bool temporal_penalty_handler::is_penalty_satisfied(
    const controller::base_controller& controller,
    const rtypes::timestep& t) const {
  bool ret = false;
  lock_rd(&m_list_mtx);
  auto it = penalty_find(controller, false);
  if (it != m_penalty_list.end()) {
    ret = it->penalty_satisfied(t);
  }
  unlock_rd(&m_list_mtx);
  return ret;
} /* is_penalty_satisfied() */

bool temporal_penalty_handler::is_serving_penalty(
    const controller::base_controller& controller,
                   bool lock) const {
  maybe_lock_rd(&m_list_mtx, lock);
  auto it = penalty_find(controller, false);
  bool ret = m_penalty_list.end() != it;
  maybe_unlock_rd(&m_list_mtx, lock);
  return ret;
} /* is_serving_penalty() */

temporal_penalty_handler::const_iterator_type
temporal_penalty_handler::penalty_find(const controller::base_controller& controller,
                                       bool lock) const {
  maybe_lock_rd(&m_list_mtx, lock);
  auto it = std::find_if(m_penalty_list.begin(),
                         m_penalty_list.end(),
                         [&](const temporal_penalty& p) {
                           return p.controller() == &controller;
                         });
  maybe_unlock_rd(&m_list_mtx, lock);
  return it;
} /* penalty_find() */

rtypes::timestep temporal_penalty_handler::penalty_finish_uniqueify(
    const rtypes::timestep& start,
    rtypes::timestep duration) const {
  for (auto it = m_penalty_list.begin(); it != m_penalty_list.end(); ++it) {
    if (it->start_time() + it->penalty() == start + duration) {
      duration += 1;
      it = m_penalty_list.begin();
    }
  } /* for(i..) */
  return duration;
} /* penalty_finish_uniqueify() */

NS_END(tv, cosm);
