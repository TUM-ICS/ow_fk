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

#include <ow_rbdl/rbdl.h>
#include <ow_rbdl/urdfreader/urdfreader.h>
#include <ow_fk/robot_model.h>

namespace ow_fk {

RobotModel::RobotModel()
{
}

bool RobotModel::init(const std::string& urdfFileName, const bool verbose)
{
  // Without floating base
  if(!ow_rbdl::Addons::URDFReadFromFile(
      urdfFileName.c_str(), &rbdl_model_, false, verbose))
  {
    ROS_ERROR_STREAM("Error loading RobotModel from urdf file "
      << urdfFileName);
    return false;
  }
  return true;
}

bool RobotModel::init( const ow::Parameter& parameter, const bool verbose)
{
  // Without floating base
  std::string urdfstr;
  parameter.get("robot_description", urdfstr);

  if(!ow_rbdl::Addons::URDFReadFromString(
       urdfstr.c_str(), &rbdl_model_, false, verbose) )
  {
    ROS_ERROR_STREAM("Error loading RobotModel from robot_description parameter");
    return false;
  }
  return true;
}

unsigned int RobotModel::getBodyPartId(const std::string& name)
{
  return rbdl_model_.GetBodyId(name.c_str());
}

void RobotModel::updateFK(const ow::JointState& q)
{
  ow_rbdl::UpdateKinematics(rbdl_model_, q.q(), q.qP(), q.qPP() );
}

//------------------------------------------------------------------------------

ow_rbdl::Math::Matrix3d RobotModel::roationWrtBase(const BodyPart& body)
{
  // rbdl gives the rotation R_base_body, hence we need the transpose
  return ow_rbdl::CalcBodyWorldOrientation(
    rbdl_model_, dummy_q_, body.id(), false).transpose();
}

ow_rbdl::Math::Vector3d RobotModel::positionWrtBase(const BodyPart& body)
{
  // obtain the body origin by feeding a zero point to rbdl
  return ow_rbdl::CalcBodyToBaseCoordinates(
    rbdl_model_, dummy_q_, body.id(), ow::Vector3::Zero(), false);
}

ow_rbdl::Math::Vector3d RobotModel::linearVeloctiyWrtBase(const BodyPart& body)
{
  // obtain the body origin velocity by feeding a zero point to rbdl
  return ow_rbdl::CalcPointVelocity(
    rbdl_model_, dummy_q_, dummy_q_, body.id(), ow::Vector3::Zero(), false);
}

ow_rbdl::Math::SpatialVector RobotModel::spatialVeloctiyWrtBase(const BodyPart& body)
{
  // obtain the body origin velocity by feeding a zero point to rbdl
  return ow_rbdl::CalcPointVelocity6D(
    rbdl_model_, dummy_q_, dummy_q_, body.id(), ow::Vector3::Zero(), false);
}

ow_rbdl::Math::SpatialVector RobotModel::spatialAccelerationWrtBase(const BodyPart& body)
{
  // obtain the body origin acceleration by feeding a zero point to rbdl
  return ow_rbdl::CalcPointAcceleration6D(
    rbdl_model_, dummy_q_, dummy_q_, dummy_q_, body.id(), ow::Vector3::Zero(), false);
}


ow_rbdl::Math::SpatialTransform RobotModel::spatialTransformationWrtBase(
  const BodyPart& body)
{
  ow_rbdl::Math::SpatialTransform T_body_base;

  T_body_base.r = positionWrtBase(body);
  T_body_base.E = roationWrtBase(body);
  return T_body_base;
}

ow::CartesianPosition RobotModel::transformationWrtBase(
  const BodyPart& body)
{
  ow::CartesianPosition X_body_base;
  X_body_base.position() = positionWrtBase(body);
  X_body_base.orientation() = roationWrtBase(body);
  return X_body_base;
}

ow_rbdl::Math::Vector3d RobotModel::centerOfMassWrtBase()
{
  double mass =0.0;
  ow_rbdl::Math::Vector3d x_base;  

  ow_rbdl::Utils::CalcCenterOfMass(
    rbdl_model_,
    dummy_q_,   // q (not used)
    dummy_q_,   // qP (not used)
    NULL,       // qPP (not used)
    mass,       // mass
    x_base,     // COM position
    NULL,       // COM velocity (not used)
    NULL,       // COM acceleration (not used)
    NULL,       // COM angular momentum (not used)
    NULL,       // COM angular momentum derivative (not used)
    false);     // Update kinematics
  return x_base;
}

ow::CartesianState RobotModel::centerOfMassStateWrtBase()
{
  double mass;
  ow::CartesianState Xstate_base;
  ow_rbdl::Math::Vector3d x_base;
  ow_rbdl::Math::Vector3d xP_base;
  ow_rbdl::Math::Vector3d xPP_base;
  ow_rbdl::Math::Vector3d omega_base;
  ow_rbdl::Math::Vector3d omegaP_base;

  ow_rbdl::Utils::CalcCenterOfMass(
        rbdl_model_,
        dummy_q_,     // q (not used)
        dummy_q_,     // qP (not used)
        NULL,         // qPP (not used)
        mass,         // mass
        x_base,       // COM position
        &xP_base,     // COM velocity                                           
        &xPP_base,    // COM acceleration 
        &omega_base,  // COM angular momentum 
        &omegaP_base, // COM angular momentum derivative
        false);       // Update kinematics

  xP_base = (xP_base.array().abs() > 10).select(0, xP_base).matrix();
  xPP_base = (xPP_base.array().abs() > 100).select(0, xPP_base).matrix();

  // Important, the orientation of com point is set to identity!
  Xstate_base.pos().linear() = x_base;
  Xstate_base.vel().linear() = xP_base;
  Xstate_base.acc().linear() = xPP_base;
  Xstate_base.pos().angular().setIdentity();
  Xstate_base.vel().angular() = omega_base;
  Xstate_base.acc().angular() = omegaP_base;
  return Xstate_base;
}

//------------------------------------------------------------------------------

ow::CartesianPosition RobotModel::transformationWrtParent(
  const BodyPart& parent,
  const BodyPart& child)
{
  ow::CartesianPosition X_child_base = transformationWrtBase(child);
  ow::CartesianPosition X_parent_base = transformationWrtBase(parent);
  return X_parent_base.inverse()*X_child_base;
}

ow::CartesianVelocity RobotModel::cartesianVelocityWrtParent(
    const BodyPart& parent,
    const BodyPart& child)
{
  // spacial velocites wrt base frame
  ow_rbdl::Math::SpatialVector XP_child_base = spatialVeloctiyWrtBase(child);
  ow_rbdl::Math::SpatialVector XP_parent_base = spatialVeloctiyWrtBase(parent);

  // the relative transformation contains 
  // the rotation from base to parent frame
  // and the translation between child to parent expressed in the base frame
  ow_rbdl::Math::SpatialTransform T_rel;
  T_rel.E = roationWrtBase(parent).transpose();
  T_rel.r = positionWrtBase(child) - positionWrtBase(parent);

  // veloctiy between child and parent expressed in parent frame
  // R_b_p*(delta V_b) - R_b_p * [delta r_b] x omega_b
  ow::SpatialVector XP_child_parent_parent = 
    T_rel.apply(XP_child_base - XP_parent_base);

  // implicitely converts spatial to cartesian
  return XP_child_parent_parent;
}

ow::CartesianAcceleration RobotModel::cartesianAccelerationWrtParent(
    const BodyPart& parent,
    const BodyPart& child)
{
  // spacial velocites wrt base frame
  ow_rbdl::Math::SpatialVector XPP_child_base = spatialAccelerationWrtBase(child);
  ow_rbdl::Math::SpatialVector XPP_parent_base = spatialAccelerationWrtBase(parent);

  // the relative transformation contains 
  // the rotation from base to parent frame
  // and the translation between child to parent expressed in the base frame
  ow_rbdl::Math::SpatialTransform T_rel;
  T_rel.E = roationWrtBase(parent).transpose();
  T_rel.r = positionWrtBase(child) - positionWrtBase(parent);

  // veloctiy between child and parent expressed in parent frame
  // R_b_p*(delta V_b) - R_b_p * [delta r_b] x omega_b
  ow::SpatialVector XPP_child_parent_parent = 
    T_rel.inverse().apply(XPP_child_base - XPP_parent_base);

  // implicitely converts spatial to cartesian
  return XPP_child_parent_parent;
}

ow::CartesianState RobotModel::cartesianStateWrtParent(
  const BodyPart& parent,
  const BodyPart& child)
{
  ow::CartesianState Xstate_child_parent;
  Xstate_child_parent.X() = transformationWrtParent(parent, child);
  Xstate_child_parent.XP() = cartesianVelocityWrtParent(parent, child);
  Xstate_child_parent.XPP() = cartesianAccelerationWrtParent(parent, child);
  return Xstate_child_parent;
}

//------------------------------------------------------------------------------

ow_rbdl::Math::Vector3d RobotModel::centerOfMassWrtParent(
  const BodyPart& parent)
{
  // transformation of parent to base
  ow::CartesianPosition X_parent_base = transformationWrtBase(parent);
  return X_parent_base.inverse()*static_cast<ow::Vector3>(centerOfMassWrtBase());
}

ow::CartesianState RobotModel::centerOfMassCartesianStateWrtParent(
  const BodyPart& parent)
{
  // linear state com in base frame
  // [x_com_base, xP_com_base, xPP_com_base]
  ow::CartesianState Xstate_com_base = centerOfMassStateWrtBase();

  // velocity and acceleration of com frame in base
  ow_rbdl::Math::SpatialVector XP_com_base, XPP_com_base;
  XP_com_base << 0,0,0, Xstate_com_base.vel().linear();
  XPP_com_base << 0,0,0, Xstate_com_base.acc().linear();

  // velocity and acceleration of parent frame in base
  ow_rbdl::Math::SpatialVector XP_parent_base = spatialVeloctiyWrtBase(parent);
  ow_rbdl::Math::SpatialVector XPP_parent_base = spatialAccelerationWrtBase(parent);

  // transformation of base wrt parent frame
  ow_rbdl::Math::SpatialTransform T_base_parent
    = spatialTransformationWrtBase(parent).inverse();

  // the relative transformation contains 
  // the rotation from base to parent frame
  // and the translation between com to parent expressed in the base frame
  ow_rbdl::Math::SpatialTransform T_rel;
  T_rel.E = T_base_parent.E;
  T_rel.r = Xstate_com_base.pos().linear() - positionWrtBase(parent);

  // veloctiy between com and parent expressed in parent frame
  // R_b_p*(delta V_b) - R_b_p * [delta r_b] x omega_b
  ow::SpatialVector XP_com_parent = T_rel.apply(XP_com_base - XP_parent_base);
  ow::SpatialVector XPP_com_parent = T_rel.apply(XPP_com_base - XPP_parent_base);

  // as cartesian state
  ow::CartesianState Xstate_com_parent;
  // linear position is com position expressed in parent frame
  // R_b_p*(x_com_b - x_p_b)
  Xstate_com_parent.X().linear() = T_base_parent.E*T_rel.r;
  // orientation is set the same as parent orientation 
  Xstate_com_parent.X().orientation() = T_base_parent.E;

  // velocity, acceleration
  Xstate_com_parent.XP() = XP_com_parent;
  Xstate_com_parent.XPP() = XPP_com_parent;

  return Xstate_com_parent;
}

}
