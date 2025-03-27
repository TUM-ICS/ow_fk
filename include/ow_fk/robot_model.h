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

#ifndef OPEN_WALKER_FK_ROBOT_MODEL_H
#define OPEN_WALKER_FK_ROBOT_MODEL_H

#include <ros/ros.h>
#include <ow_core/types.h>
#include <ow_rbdl/Model.h>
#include <ow_fk/body_part.h>
#include <ow_core/common/parameter.h>


namespace ow_fk
{

/*!
 * \brief The RobotModel class
 *
 * This class calculates the forward kinematics of the robot. It is a wrapper
 * for the RBDL library offering simplified access functions for the required
 * calculations of the Open Walker controllers.
 *
 * The RBDL library https://rbdl.github.io/ is heavily inspired by the pseudo
 * code of the book "Rigid Body Dynamics Algorithms" of Roy Featherstone.
 */
class RobotModel
{

protected:

  ow_rbdl::Model rbdl_model_; //!< RBDL RobotModel.

  Eigen::VectorXd dummy_q_;   //!< empty q vector needed to call rbdl functions.

public:

  /*!
   * \brief RobotModel
   *    Default constructor.
   */
  RobotModel();

  /*!
   * \brief Initialize RobotModel from URDF file.
   *
   * \param urdfFileName
   *    Full path to URDF file.
   *
   * \param verbose
   *    Show constructed rbdl RobotModel.
   *
   * \return
   *    true on success.
   */
  bool init( const std::string& urdfFileName,
             const bool verbose = false );

  /*!
   * \brief Initialize RobotModel from parameters.
   *
   * \param parameter
   *    Main parameters. It must contain robot_description.
   *
   * \param verbose
   *    Show constructed rbdl RobotModel.
   *
   * \return
   *    true on success.
   */
  bool init(const ow::Parameter& parameter, const bool verbose = false );

  /*!
   * \brief Find the rbdl id of a body part known by name.
   *
   * \param name
   *    Name of the bodypart.
   *
   * \return
   *    RBDL id of the bodypart. Notice that for fixed bodies the IDs start
   *    after the fixed body discriminator index. For more details see the RBDL
   *    library documentation.
   */
  unsigned int getBodyPartId(const std::string& name);

  /*!
   * \brief Update kinematics.
   *
   * \param q
   *    Current JointState.
   */
  void updateFK( const ow::JointState& q );

  //----------------------------------------------------------------------------

  // roation of body wrt rbdl base frame
  ow_rbdl::Math::Matrix3d roationWrtBase(const BodyPart& body);

  // position of body wrt rbdl base frame
  ow_rbdl::Math::Vector3d positionWrtBase(const BodyPart& body);

  // spatial transformation of body wrt rbdl base frame
  ow_rbdl::Math::SpatialTransform spatialTransformationWrtBase(const BodyPart& body);

  // cartesian transformation of body wrt rbdl base frame
  ow::CartesianPosition transformationWrtBase(const BodyPart& body);

  // linear velocity of body wrt rbdl base frame
  ow_rbdl::Math::Vector3d linearVeloctiyWrtBase(const BodyPart& body);

  // 6d spatial velocity of body wrt rbdl base frame
  ow_rbdl::Math::SpatialVector spatialVeloctiyWrtBase(const BodyPart& body);

  // 6d spatial acceleration of body wrt rbdl base frame
  ow_rbdl::Math::SpatialVector spatialAccelerationWrtBase(const BodyPart& body);

  // com position of model in rbdl base frame
  ow_rbdl::Math::Vector3d centerOfMassWrtBase();

  // com position, velocity, acceleration of model in rbdl base frame
  ow::CartesianState centerOfMassStateWrtBase();

  //----------------------------------------------------------------------------

  // cartesian transformation of child body wrt parent frame
  ow::CartesianPosition transformationWrtParent(
    const BodyPart& parent,
    const BodyPart& child);

  // cartesian velocity of child body wrt parent frame
  // = relative velocity child wrt to parent expressed in parent frame
  ow::CartesianVelocity cartesianVelocityWrtParent(
    const BodyPart& parent,
    const BodyPart& child);

  // cartesian acceleration of child body wrt parent frame
  // = relative acceleration child wrt to parent expressed in parent frame
  ow::CartesianAcceleration cartesianAccelerationWrtParent(
    const BodyPart& parent,
    const BodyPart& child);

  // cartesian state of child body wrt parent frame
  // = relative velocity, acceleration child wrt to parent expressed in parent frame
  ow::CartesianState cartesianStateWrtParent(
    const BodyPart& parent,
    const BodyPart& child);

  // com position wrt to parent frame
  ow_rbdl::Math::Vector3d centerOfMassWrtParent(
    const BodyPart& parent);

  // com position wrt to parent frame
  // = relative velocity, acceleration com wrt to parent expressed in parent frame
  ow::CartesianState centerOfMassCartesianStateWrtParent(
    const BodyPart& parent);
};

}

#endif // OPEN_WALKER_FK_ROBOT_MODEL_H
