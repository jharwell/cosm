/**
 * \file argos_pd_adaptor.cpp
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
#include "cosm/pal/tv/argos_pd_adaptor.hpp"

#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>

#include "cosm/tv/config/population_dynamics_config.hpp"
#include "cosm/pal/operations/argos_robot_malfunction.hpp"
#include "cosm/pal/operations/argos_robot_repair.hpp"
#include "cosm/pal/argos_controller2D_adaptor.hpp"
#include "cosm/pal/argos_controllerQ3D_adaptor.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/arena/base_arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, tv);

using op_result = ctv::population_dynamics::op_result;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
template<typename TControllerType>
argos_pd_adaptor<TControllerType>::argos_pd_adaptor(
    const ctv::config::population_dynamics_config* config,
    cpal::argos_sm_adaptor* const sm,
    carena::base_arena_map* map,
    env_dynamics_type* envd,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.pal.tv.argos_pd_adaptor"),
      population_dynamics(config,
                          sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size(),
                          sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size(),
                          rng),
      mc_sm(sm),
      m_map(map),
      m_envd(envd),
      m_rng(rng),
      m_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template<typename TControllerType>
op_result argos_pd_adaptor<TControllerType>::robot_kill(void) {
  size_t active_pop = swarm_active_population();
  size_t total_pop = swarm_total_population();

  if (0 == total_pop) {
    ER_WARN("Not killing robot: total_pop=%zu active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  auto* controller = kill_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find kill victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  rtypes::type_uuid id = controller->entity_id();
  ER_INFO("Found kill victim with ID=%d", id.v());

  std::string name = kARGoSRobotNamePrefix + rcppsw::to_string(id);

  /*
   * Before removing the robot, perform any project-specific robot cleanup.
   */
  pre_kill_cleanup(controller);

  /* remove controller from any applied environmental variances */
  m_envd->unregister_controller(*controller);

  /* remove robot from ARGoS */
  ER_INFO("Remove entity %s", name.c_str());
  m_sm->RemoveEntity(name);

  /* killing a robot reduces both the active and total populations */
  return {id,
        m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size(),
        m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size()};
} /* robot_kill() */

template<typename TControllerType>
op_result argos_pd_adaptor<TControllerType>::robot_add(int max_pop,
                                                       const rtypes::type_uuid& id) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (max_pop != -1 && total_pop >= static_cast<size_t>(max_pop)) {
    ER_INFO("Not adding new robot %s%d: max_pop=%d reached",
            kARGoSRobotNamePrefix,
            id.v(),
            max_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    if (robot_attempt_add(id)) {
      /* adding a new robot increases both the total and active populations */
      return {id,
            m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size(),
            m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).size()};
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to add new robot to simulation");
  return {rtypes::constants::kNoUUID, total_pop, active_pop};
} /* robot_add() */

template<typename TControllerType>
op_result argos_pd_adaptor<TControllerType>::robot_malfunction(void) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (0 == total_pop || 0 == active_pop) {
    ER_WARN("Cannot force robot malfunction: total_pop=%zu, active_pop=%zu",
            total_pop, active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }

  auto* controller = malfunction_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find malfunction victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return {rtypes::constants::kNoUUID, total_pop, active_pop};
  }
  ER_INFO("Found malfunction victim with ID=%d,n_repairing=%zu",
          controller->entity_id().v(),
          repair_queue_status().size);

  cpops::argos_robot_malfunction visitor;
  visitor.visit(*controller);

  /*
   * @todo: Once the ability to temporarily remove robots from ARGoS by removing
   * it from physics engines is added, do that here.
   */
  return {controller->entity_id(), total_pop, active_pop - 1};
} /* robot_malfunction() */

template<typename TControllerType>
op_result argos_pd_adaptor<TControllerType>::robot_repair(const rtypes::type_uuid& id) {

  ER_INFO("Robot with ID=%d repaired, n_repairing=%zu",
          id.v(),
          repair_queue_status().size);
  std::string name = kARGoSRobotNamePrefix + rcppsw::to_string(id);
  cpops::argos_robot_repair visitor;
  auto* entity = dynamic_cast<argos::CFootBotEntity*>(
      &m_sm->GetSpace().GetEntity(name));
  auto* controller = static_cast<TControllerType*>(
      &entity->GetControllableEntity().GetController());
  visitor.visit(*controller);

  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  /*
   * @todo: Once the ability to restore temporarily removed robots from
   * simulation is added, do that here.
   */
  return {controller->entity_id(), total_pop, active_pop + 1};
} /* robot_repair() */

template<typename TControllerType>
TControllerType* argos_pd_adaptor<TControllerType>::malfunction_victim_locate(size_t total_pop) const {
  auto range = rmath::rangei(0, total_pop - 1);

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).begin();
    std::advance(it, m_rng->uniform(range));
    auto* entity = argos::any_cast<argos::CFootBotEntity*>(it->second);
    auto* controller = static_cast<TControllerType*>(
        &entity->GetControllableEntity().GetController());
    if (!currently_repairing(controller->entity_id())) {
      return controller;
    }
  } /* for(i..) */
  return nullptr;
} /* malfunction_victim_locate() */

template<typename TControllerType>
TControllerType* argos_pd_adaptor<TControllerType>::kill_victim_locate(size_t total_pop) const {
    auto range = rmath::rangei(0, total_pop - 1);
  argos::CFootBotEntity* entity;
  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_sm->GetSpace().GetEntitiesByType(kARGoSRobotType).begin();
    std::advance(it, m_rng->uniform(range));
    entity = argos::any_cast<argos::CFootBotEntity*>(it->second);
    auto* controller = static_cast<TControllerType*>(
        &entity->GetControllableEntity().GetController());
    if (!already_killed(controller->entity_id())) {
      return controller;
      break;
    }
  } /* for(i..) */
  return nullptr;
} /* kill_victim_locate() */

template<typename TControllerType>
bool argos_pd_adaptor<TControllerType>::robot_attempt_add(const rtypes::type_uuid& id) {
  /*
   * Give 2.0 buffer around the edges of the arena so that robots are not too
   * close to the boundaries of physics engines, which can cause "no engine can
   * house entity" exceptions in rare cases otherwise.
   */
  rmath::ranged xrange(2.0, mc_sm->arena_map()->xrsize() - 2.0);
  rmath::ranged yrange(2.0, mc_sm->arena_map()->yrsize() - 2.0);
  argos::CFootBotEntity* fb = nullptr;

  auto x = m_rng->uniform(xrange);
  auto y = m_rng->uniform(yrange);

  /*
   * You CANNOT first create the entity, then attempt to move it to a
   * collision free location within ARGoS when there are multiple physics
   * engines used--you always get an exception thrown. The only way to ensure
   * correct operation is to pass the desired location to the entity constructor
   * BEFORE calling AddEntity(). This is suboptimal, because it involves
   * potentially a lot of dynamic memory management that can slow things down,
   * but it is required. See #623.
   *
   * This is a @bug in ARGoS, and so this code can be reverted to something like
   * what is was originally once this is fixed in the ARGoS master. Diffing the
   * branch for #623 against the previous commit should show the changes.
   */
  try {
    /* ick raw pointers--thanks ARGoS... */
    fb = new argos::CFootBotEntity(kARGoSRobotNamePrefix + rcppsw::to_string(id),
                                   kARGoSControllerXMLId,
                                   argos::CVector3(x, y, 0.0));
    m_sm->AddEntity(*fb);
    ER_INFO(
        "Added entity %s attached to physics engine %s at %s",
        fb->GetId().c_str(),
        fb->GetEmbodiedEntity().GetPhysicsModel(0).GetEngine().GetId().c_str(),
        rmath::vector2d(x, y).to_str().c_str());

    /* Register controller for environmental variances */
    auto* controller = static_cast<TControllerType*>(
        &fb->GetControllableEntity().GetController());

    m_envd->register_controller(*controller);

    return true;
  } catch (argos::CARGoSException& e) {
    if (nullptr != fb) {
      delete fb; /* ick raw pointers--thanks ARGoS... */
    }
    ER_TRACE("Failed to place new robot %s at %s",
             fb->GetId().c_str(),
             rmath::vector2d(x, y).to_str().c_str());
    return false;
  }
} /* robot_attempt_add() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class argos_pd_adaptor<::cosm::pal::argos_controller2D_adaptor>;
template class argos_pd_adaptor<::cosm::pal::argos_controllerQ3D_adaptor>;

NS_END(tv, pal, cosm);
