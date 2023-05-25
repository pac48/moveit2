/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Jens Petit */

#include <moveit/collision_detection_neural/collision_env_neural.h>
#include <moveit/collision_detection_neural/collision_detector_allocator_neural.h>
#include <moveit/collision_detection_neural/collision_common.h>

#include <moveit/collision_detection_neural/neural_compat.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace collision_detection
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection_neural.collision_env_neural");
const std::string CollisionDetectorAllocatorNeural::NAME("Neural");

CollisionEnvNeural::CollisionEnvNeural(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
}
CollisionEnvNeural::CollisionEnvNeural(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                                       double padding, double scale)
  : CollisionEnv(model, world, padding, scale)
{
}
CollisionEnvNeural::CollisionEnvNeural(const CollisionEnvNeural& other, const WorldPtr& world) : CollisionEnv(other, world)
{
}
CollisionEnvNeural::~CollisionEnvNeural()
{
}
void CollisionEnvNeural::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state) const
{
}
void CollisionEnvNeural::checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                                            const moveit::core::RobotState& state,
                                            const AllowedCollisionMatrix& acm) const
{
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state) const
{
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2,
                                             const AllowedCollisionMatrix& acm) const
{
}
void CollisionEnvNeural::distanceSelf(const DistanceRequest& req, DistanceResult& res,
                                      const moveit::core::RobotState& state) const
{
}
void CollisionEnvNeural::distanceRobot(const DistanceRequest& req, DistanceResult& res,
                                       const moveit::core::RobotState& state) const
{
}
void CollisionEnvNeural::setWorld(const WorldPtr& world)
{
  CollisionEnv::setWorld(world);
}
void CollisionEnvNeural::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  CollisionEnv::updatedPaddingOrScaling(links);
}
void CollisionEnvNeural::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const moveit::core::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
}
void CollisionEnvNeural::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const AllowedCollisionMatrix* acm) const
{
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2) const
{
}
void CollisionEnvNeural::notifyObjectChange(const CollisionEnv::ObjectConstPtr& obj, World::Action action)
{
}
}  // end of namespace collision_detection
