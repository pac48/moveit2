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
#include "geometric_shapes/shapes.h"

#include "cuda_collision_checking/cuda_collision_checking.hpp"
#include "tiny-cuda-nn/cpp_api.h"
#include <rclcpp/logger.hpp>

namespace collision_detection
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_collision_detection_neural.collision_env_neural");
const std::string CollisionDetectorAllocatorNeural::NAME("Neural");

//void InitCommon(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
//{
//
//  auto links = model->getLinkModelsWithCollisionGeometry();
//  for (const auto& link : links)
//  {
//    for (std::size_t j{ 0 }; j < link->getShapes().size(); ++j)
//    {
//      auto tmp = link->getShapes()[j];  // cast to shape to get verts
//      if (tmp->type == shapes::MESH)
//      {
//        auto mesh = std::dynamic_pointer_cast<const shapes::Mesh>(tmp);  // cast to shape to get verts
//      }
//      else
//      {
//        RCLCPP_ERROR(LOGGER, "A non-MESH was found!!");
//      }
//      //      tmp->global_link_transforms_
//
//      //      mesh->triangles
//      //      mesh->vertices
//      //      mesh->vertex_count
//    }
//  }
//}

CollisionEnvNeural::CollisionEnvNeural(const moveit::core::RobotModelConstPtr& model, double padding, double scale)
  : CollisionEnv(model, padding, scale)
{
//  stream_ptr_ = new cudaStream_t();



//  auto links = model->getLinkModelsWithCollisionGeometry();
//  for (const auto& link : links)
//  {
//    for (std::size_t j{ 0 }; j < link->getShapes().size(); ++j)
//    {
//      auto tmp = link->getShapes()[j];  // cast to shape to get verts
//      if (tmp->type == shapes::MESH)
//      {
//        auto mesh = std::dynamic_pointer_cast<const shapes::Mesh>(tmp);  // cast to shape to get verts
//      }
//      else
//      {
//        RCLCPP_ERROR(LOGGER, "A non-MESH was found!!");
//      }
//      //      tmp->global_link_transforms_
//
//      //      mesh->triangles
//      //      mesh->vertices
//      //      mesh->vertex_count
//    }
//  }




  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}
CollisionEnvNeural::CollisionEnvNeural(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world,
                                       double padding, double scale)
  : CollisionEnv(model, world, padding, scale)
{
//  stream_ptr_ = new cudaStream_t();


//  auto links = model->getLinkModelsWithCollisionGeometry();
//  for (const auto& link : links)
//  {
//    for (std::size_t j{ 0 }; j < link->getShapes().size(); ++j)
//    {
//      auto tmp = link->getShapes()[j];  // cast to shape to get verts
//      if (tmp->type == shapes::MESH)
//      {
//        auto mesh = std::dynamic_pointer_cast<const shapes::Mesh>(tmp);  // cast to shape to get verts
//      }
//      else
//      {
//        RCLCPP_ERROR(LOGGER, "A non-MESH was found!!");
//      }
//    }
//  }


  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}

CollisionEnvNeural::CollisionEnvNeural(const CollisionEnvNeural& other, const WorldPtr& world)
  : CollisionEnv(other, world)
{
  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}
CollisionEnvNeural::~CollisionEnvNeural()
{
  if (stream_active_)
  {
//    cudaStreamDestroy(*stream_ptr_);
//    delete stream_ptr_;
    stream_active_ = false;
  }
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
  for (const auto& link : robot_model_->getLinkModelsWithCollisionGeometry())
  {
    auto T = state.getGlobalLinkTransform(link->getName());
    //    T apply to verts
  }
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state,
                                             const AllowedCollisionMatrix& acm) const
{
  int o = 0;
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2,
                                             const AllowedCollisionMatrix& acm) const
{
  int o = 0;
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

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(
      [this](const World::ObjectConstPtr& object, World::Action action) { notifyObjectChange(object, action); });
}
void CollisionEnvNeural::updatedPaddingOrScaling(const std::vector<std::string>& links)
{
  CollisionEnv::updatedPaddingOrScaling(links);
}
void CollisionEnvNeural::checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                  const moveit::core::RobotState& state,
                                                  const AllowedCollisionMatrix* acm) const
{
  int o = 0;
}
void CollisionEnvNeural::checkRobotCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                                   const moveit::core::RobotState& state,
                                                   const AllowedCollisionMatrix* acm) const
{
  int o = 0;
}
void CollisionEnvNeural::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                             const moveit::core::RobotState& state1,
                                             const moveit::core::RobotState& state2) const
{
  int o = 0;
}
void CollisionEnvNeural::notifyObjectChange(const CollisionEnv::ObjectConstPtr& obj, World::Action action)
{
  if (action == World::DESTROY)
  {
    // delete object
  }
  else if (action == World::ADD_SHAPE || action == (World::CREATE + World::ADD_SHAPE))
  {
    assert(obj->shapes_.size() == 1);
    assert(obj->shapes_[0]->type == shapes::ShapeType::NEURAL);
    auto n_shape = std::dynamic_pointer_cast<const shapes::Neural>(obj->shapes_[0]);

    if (!stream_active_)
    {
//      cudaStreamCreate(stream_ptr_);
      stream_active_ = true;
    }

    constexpr uint32_t n_input_dims = 3;
    constexpr uint32_t n_output_dims = 1;
    trainable_model_ = tcnn::cpp::create_trainable_model(n_input_dims, n_output_dims, n_shape->config);

//    params_ = trainable_model_->params();
//    cudaMemcpy(params_, n_shape->weights.data(), n_shape->weights.size() * sizeof(float), cudaMemcpyDeviceToHost);

  }
}
}  // end of namespace collision_detection
