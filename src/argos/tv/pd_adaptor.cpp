/**
 * \file pd_adaptor.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/tv/pd_adaptor.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/argos/operations/robot_malfunction.hpp"
#include "cosm/argos/operations/robot_repair.hpp"
#include "cosm/hal/robot.hpp"
#include "cosm/pal/argos/controller/adaptor2D.hpp"
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/tv/config/population_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::argos::tv {

using op_result = ctv::population_dynamics::op_result;

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
template <typename TController>
pd_adaptor<TController>::pd_adaptor(
    const ctv::config::population_dynamics_config* config,
    cpargos::swarm_manager_adaptor* const sm,
    env_dynamics_type* envd,
    const rmath::vector2d& arena_dim,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.pal.tv.argos.pd_adaptor"),
      population_dynamics(
          config,
          sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size(),
          sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size(),
          rng),
      mc_sm(sm),
      mc_arena_dim(arena_dim),
      m_envd(envd),
      m_rng(rng),
      m_sm(sm) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <typename TController>
op_result pd_adaptor<TController>::robot_kill(void) {
  size_t active_pop = swarm_active_population();
  size_t total_pop = swarm_total_population();

  if (0 == total_pop) {
    ER_WARN(
        "Not killing robot: total_pop=%zu active_pop=%zu", total_pop, active_pop);
    return { rtypes::constants::kNoUUID, total_pop, active_pop };
  }
  auto* controller = kill_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find kill victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return { rtypes::constants::kNoUUID, total_pop, active_pop };
  }
  rtypes::type_uuid id = controller->entity_id();
  ER_INFO("Found kill victim with ID=%d", id.v());

  std::string name = cpal::kRobotNamePrefix + rcppsw::to_string(id);

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
  return { id,
           m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size(),
           m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size() };
} /* robot_kill() */

template <typename TController>
op_result pd_adaptor<TController>::robot_add(int max_pop,
                                             const rtypes::type_uuid& id) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (max_pop != -1 && total_pop >= static_cast<size_t>(max_pop)) {
    ER_INFO("Not adding new robot %s%d: max_pop=%d reached",
            cpal::kRobotNamePrefix.c_str(),
            id.v(),
            max_pop);
    return { rtypes::constants::kNoUUID, total_pop, active_pop };
  }

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    if (robot_attempt_add(id)) {
      /* adding a new robot increases both the total and active populations */
      return { id,
               m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size(),
               m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size() };
    }
  } /* for(i..) */
  ER_FATAL_SENTINEL("Unable to add new robot to simulation");
  return { rtypes::constants::kNoUUID, total_pop, active_pop };
} /* robot_add() */

template <typename TController>
op_result pd_adaptor<TController>::robot_malfunction(void) {
  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  if (0 == total_pop || 0 == active_pop) {
    ER_WARN("Cannot force robot malfunction: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return { rtypes::constants::kNoUUID, total_pop, active_pop };
  }

  auto* controller = malfunction_victim_locate(total_pop);

  if (nullptr == controller) {
    ER_WARN("Unable to find malfunction victim: total_pop=%zu, active_pop=%zu",
            total_pop,
            active_pop);
    return { rtypes::constants::kNoUUID, total_pop, active_pop };
  }
  ER_INFO("Found malfunction victim with ID=%d,n_repairing=%zu",
          controller->entity_id().v(),
          repair_queue_status().size);

  cargos::operations::robot_malfunction visitor;
  visitor.visit(*controller);

  /*
   * @todo: Once the ability to temporarily remove robots from ARGoS by removing
   * it from physics engines is added, do that here.
   */
  return { controller->entity_id(), total_pop, active_pop - 1 };
} /* robot_malfunction() */

template <typename TController>
op_result pd_adaptor<TController>::robot_repair(const rtypes::type_uuid& id) {
  ER_INFO("Robot with ID=%d repaired, n_repairing=%zu",
          id.v(),
          repair_queue_status().size);
  std::string name = cpal::kRobotNamePrefix + rcppsw::to_string(id);
  cargos::operations::robot_repair visitor;
  auto* entity = dynamic_cast<chal::robot*>(&m_sm->GetSpace().GetEntity(name));
  auto* controller =
      static_cast<TController*>(&entity->GetControllableEntity().GetController());
  visitor.visit(*controller);

  size_t total_pop = swarm_total_population();
  size_t active_pop = swarm_active_population();

  /*
   * @todo: Once the ability to restore temporarily removed robots from
   * simulation is added, do that here.
   */
  return { controller->entity_id(), total_pop, active_pop + 1 };
} /* robot_repair() */

template <typename TController>
TController*
pd_adaptor<TController>::malfunction_victim_locate(size_t total_pop) const {
  auto range = rmath::rangei(0, total_pop - 1);

  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).begin();
    std::advance(it, m_rng->uniform(range));
    auto* entity = ::argos::any_cast<chal::robot*>(it->second);
    auto* controller = static_cast<TController*>(
        &entity->GetControllableEntity().GetController());
    if (!currently_repairing(controller->entity_id())) {
      return controller;
    }
  } /* for(i..) */
  return nullptr;
} /* malfunction_victim_locate() */

template <typename TController>
TController* pd_adaptor<TController>::kill_victim_locate(size_t total_pop) const {
  auto range = rmath::rangei(0, total_pop - 1);
  chal::robot* entity;
  for (size_t i = 0; i < kMaxOperationAttempts; ++i) {
    auto it = m_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).begin();
    std::advance(it, m_rng->uniform(range));
    entity = ::argos::any_cast<chal::robot*>(it->second);
    auto* controller = static_cast<TController*>(
        &entity->GetControllableEntity().GetController());
    if (!already_killed(controller->entity_id())) {
      return controller;
      break;
    }
  } /* for(i..) */
  return nullptr;
} /* kill_victim_locate() */

template <typename TController>
bool pd_adaptor<TController>::robot_attempt_add(const rtypes::type_uuid& id) {
  /*
   * Give 2.0 buffer around the edges of the arena so that robots are not too
   * close to the boundaries of physics engines, which can cause "no engine can
   * house entity" exceptions in rare cases otherwise.
   */
  rmath::ranged xrange(2.0, mc_arena_dim.x() - 2.0);
  rmath::ranged yrange(2.0, mc_arena_dim.y() - 2.0);
  chal::robot* robot = nullptr;

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
   * This is a \bug in ARGoS, and so this code can be reverted to something like
   * what is was originally once this is fixed in the ARGoS master. Diffing the
   * branch for FORDYCA#623 against the previous commit should show the changes.
   */
  try {
    robot = new chal::robot(cpal::kRobotNamePrefix + rcppsw::to_string(id),
                            cpal::kControllerXMLId,
                            ::argos::CVector3(x, y, 0.0),
                            ::argos::CQuaternion());
    m_sm->AddEntity(*robot);
    ER_INFO(
        "Added entity %s attached to physics engine %s at %s",
        robot->GetId().c_str(),
        robot->GetEmbodiedEntity().GetPhysicsModel(0).GetEngine().GetId().c_str(),
        rmath::vector2d(x, y).to_str().c_str());

    /* Register controller for environmental variances */
    auto* controller = static_cast<TController*>(
        &robot->GetControllableEntity().GetController());

    m_envd->register_controller(*controller);

    return true;
  } catch (::argos::CARGoSException& e) {
    delete robot; /* ick raw pointers--thanks ARGoS... */
    ER_TRACE("Failed to place new robot %s at %s",
             robot->GetId().c_str(),
             rmath::vector2d(x, y).to_str().c_str());
    return false;
  }
} /* robot_attempt_add() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class pd_adaptor<cpargos::controller::adaptor2D>;
template class pd_adaptor<cpargos::controller::adaptorQ3D>;

} /* namespace cosm::argos::tv */
