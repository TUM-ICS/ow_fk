/*! \file
 *
 * \author J. Rogelio Guadarrama-Olvera
 * \author Emmanuel Dean-Leon
 * \author Florian Bergner
 * \author Simon Armleder
 * \author Gordon Cheng
 *
 * \version 0.1
 * \date 03.05.2020
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 * #### Acknowledgment
 *  This project has received funding from the European Union‘s Horizon 2020
 *  research and innovation programme under grant agreement No 732287.
 */

#include <ow_fk/forward_kinematics.h>
#include <ow_core/math.h>

namespace ow_fk
{

  ForwardKinematics::ForwardKinematics() : 
    Base("forward_kinematic"),
    X_odom_w_(ow::CartesianPosition::Identity()),
    X_com_hip_(ow::CartesianState::Zero()),
    X_com_w_(ow::CartesianState::Zero()),
    ref_foot_(ow::FootId::RIGHT)
  {
  }

  ForwardKinematics::~ForwardKinematics()
  {
  }

  bool ForwardKinematics::init(const ow::Parameter &parameter, ros::NodeHandle &nh)
  {
    // build the configuration for the forward kinematics
    parameter_.add<std::string>("body/legs/left/base", "");
    parameter_.add<std::string>("body/legs/left/ee", "");
    parameter_.add<std::string>("body/legs/right/ee", "");
    parameter_.add<bool>("use_com");

    // load everything
    if (!parameter_.load(nh, "kinematics"))
    {
      ROS_ERROR("%s::initialize: Config loading failed.", Base::name().c_str());
      return false;
    }

    if (!robot_model_.init(parameter, false))
    {
      ROS_ERROR("%s::init: Error initalizing robot model.", Base::name().c_str());
      return false;
    }

    // init members
    parameter_.get("use_com", use_com_);
    // put inital reference on Left foot
    ref_foot_ = ow::FootId::LEFT;

    // create body parts
    std::string name;
    parameter_.get("body/legs/left/ee", name);
    body_parts_.push_back(BodyPart(name, robot_model_.getBodyPartId(name)));

    parameter_.get("body/legs/right/ee", name);
    body_parts_.push_back(BodyPart(name, robot_model_.getBodyPartId(name)));

    parameter_.get("body/legs/left/base", name);
    body_parts_.push_back(BodyPart(name, robot_model_.getBodyPartId(name)));

    // store left and right foot states in vector
    foot_states_.resize(2, ow::CartesianState::Zero());
    return true;
  }

  void ForwardKinematics::update(const ow::Flags &flags, const ow::JointState &q)
  {
    // update the complete forward dynamics
    robot_model_.updateFK(q);

    // ref foot is static and equal to odometry.
    updateRefFoot(flags.supportFoot());

    // update the swing foot
    updateOtherFootState(flags.supportFoot());

    if (use_com_)
    {
      // use the cad model based com frame as the com
      updateComState(flags.supportFoot());
    }
    else
    {
      // use the hip frame as approximation of the com
      updateHipState(flags.supportFoot());
    }
  }

  void ForwardKinematics::setRefFootState(
      const ow::Flags& flags,
      const ow::CartesianState &X_ref_w)
  {
    ref_foot_ = flags.supportFoot();

    // Reference foot must be in static contact with the ground.
    X_odom_w_ = X_ref_w.pos();
    X_odom_w_.linear().z() = 0.0;
    
    foot_states_.at(ref_foot_) = X_ref_w;
    foot_states_.at(ref_foot_).vel().setZero();
    foot_states_.at(ref_foot_).acc().setZero();
  }

  void ForwardKinematics::updateRefFootState(const BodyPartId ref_foot)
  {
    foot_states_.at(ref_foot).pos() = X_odom_w_;
    foot_states_.at(ref_foot).vel().setZero();
    foot_states_.at(ref_foot).acc().setZero();
  }

  void ForwardKinematics::updateOtherFootState(ow::FootId ref_foot)
  {
    // get other foot
    ow::FootId other_foot = ref_foot.other();

    // compute other foot wrt ref foot
    ow::CartesianState X_other_ref;
    X_other_ref = robot_model_.cartesianStateWrtParent(
        body_parts_.at(ref_foot), body_parts_.at(other_foot));

    // compute other foot wrt world
    foot_states_.at(other_foot) = ow::changeRefFrame(X_other_ref, X_odom_w_);
  }

  ow::FootId ForwardKinematics::updateRefFoot(
      const ow::FootId ref_foot)
  {
    if (ref_foot_ != ref_foot)
    {
      ow::FootId other_foot = ref_foot.other();

      X_odom_w_ = robot_model_.transformationWrtParent(
          body_parts_.at(ref_foot), body_parts_.at(other_foot));
    }
    return ref_foot;
  }

  void ForwardKinematics::updateComState(ow::FootId ref_foot)
  {
    // compute the model based com state wrt ref foot frame
    ow::CartesianState X_com_ref;
    X_com_ref = robot_model_.centerOfMassCartesianStateWrtParent(
        body_parts_.at(ref_foot));

    // transform to world frame
    X_com_w_ = ow::changeRefFrame(X_com_ref, X_odom_w_);

    // compute the com state wrt hip frame
    // assume identity rotation
    X_com_hip_ = robot_model_.centerOfMassCartesianStateWrtParent(
        body_parts_.at(HIP));
    X_com_hip_.pos().angular().setIdentity();
  }

  void ForwardKinematics::updateHipState(ow::FootId ref_foot)
  {
    // use the hip as approximation of com
    ow::CartesianState X_com_ref;
    X_com_ref = robot_model_.cartesianStateWrtParent(
        body_parts_.at(ref_foot), body_parts_.at(HIP));

    // transform to world frame
    X_com_w_ = ow::changeRefFrame(X_com_ref, X_odom_w_);

    // com to hip offset is zero, rotation is identity
    X_com_hip_ = ow::CartesianState::Zero();
  }

  const ow::CartesianState &ForwardKinematics::X_l_w() const
  {
    return foot_states_.at(LEFT_FOOT);
  }

  const ow::CartesianState &ForwardKinematics::X_r_w() const
  {
    return foot_states_.at(RIGHT_FOOT);
  }

  const ow::CartesianState &ForwardKinematics::X_com_w() const
  {
    return X_com_w_;
  }

  const ow::CartesianState &ForwardKinematics::X_com_hip() const
  {
    return X_com_hip_;
  }

} // namespace ow_fk
