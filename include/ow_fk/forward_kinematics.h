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

#ifndef OPEN_WALKER_FK_H
#define OPEN_WALKER_FK_H

#include <ow_core/types.h>
#include <ow_fk/body_part.h>
#include <ow_fk/robot_model.h>

#include <ow_core/interfaces/i_forward_kinematics.h>

/*!
 * \brief Open Walker forward kinematics module namespace. These classes
 * implement the forward kinematics and dynamics models for the robot.
 */
namespace ow_fk
{

/*!
 * \brief The ForwardKinematics class
 *
 * This class calculates the forward kinematics of the robot limbs 
 * (left foot, right foot and hip/com).
 * 
 * The position, velocity, acceleration of the cartesian states is computed 
 * wrt to the world frame.
 * The current robot odometry is stored in X_odom_w_
 */
class ForwardKinematics : 
  public ow::IForwardKinematics
{
public:
  typedef ow::IForwardKinematics Base;

  enum BodyPartId
  {
    LEFT_FOOT,      //!< Left foot
    RIGHT_FOOT,     //!< Right foot
    HIP,            //!< Hip
  };

protected:
  ow::Parameter parameter_;               //!< configuration
  bool use_com_;                          //!< use the model based com

  std::vector<BodyPart> body_parts_;    //!< Vector of body parts.
  RobotModel robot_model_;              //!< Kinematic and dynamic robot model.
  ow::FootId ref_foot_;                 //!< reference foot (fixed to the ground) 

  // cartesian states of the robot
  std::vector<ow::CartesianState> foot_states_;   //!< states of left/ right foot
  ow::CartesianState X_com_w_;                    //!< com wrt world
  ow::CartesianState X_com_hip_;                  //!< com wrt hip

  /*!
   * \brief Odometry of the robot. 
   * 
   * Stores the transformation of the robot wrt world.
   */
  ow::CartesianPosition X_odom_w_;

public:
  /*!
  * \brief ForwardKinematics
  *    Default constructor.
  */
  ForwardKinematics();

  /*!
   * \brief Desturctor
   */
  virtual ~ForwardKinematics();

  /*!
  * \brief Update the Cartesian States of the feet and the center of mass wrt
  *    world frame.
  * 
  * \param flags
  *
  * \param q
  *    Current JointState.
  */
  void update(const ow::Flags &flags, const ow::JointState &q);

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the leftFoot wrt world frame.
  */
  virtual const ow::CartesianState& X_l_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the rightFoot wrt world frame.
  */
  virtual const ow::CartesianState& X_r_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the CoM wrt world frame.
  */
  virtual const ow::CartesianState& X_com_w() const;

  /*!
  * \brief Output port function.
  *
  * \return
  *    CartesianState of the CoM wrt X_com_hip.
  */
  virtual const ow::CartesianState& X_com_hip() const;

  /*!
   * \brief setRefFootState
   * \param flags
   * \param X_ref_w
   */
  void setRefFootState(const ow::Flags& flags,
                       const ow::CartesianState& X_ref_w );

protected:
  /*!
  * \brief Initialization of Forward Kinematics module.
  *
  * \return
  *    true on success.
  */
  virtual bool init(const ow::Parameter& parameter, ros::NodeHandle& nh);

private:
  /*!
  * \brief Update the CartesianState of the reference/suporting foot wrt world
  *    frame.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  */
  void updateRefFootState(const BodyPartId ref_foot);

  /*!
  * \brief Update the CartesianState of the CoM wrt world frame.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  */
  void updateComState( ow::FootId ref_foot);

  /*!
  * \brief Update the CartesianState of the Hip wrt world frame.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  */
  void updateHipState( ow::FootId ref_foot);

  /*!
  * \brief Update the CartesianState of the foot which is not the
  *    reference/suporting foot wrt world frame.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  */
  void updateOtherFootState( ow::FootId ref_foot);

  /*!
  * \brief Update the odometry if the reference/suporting foot changed.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  *
  * \return
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  */
  ow::FootId updateRefFoot(const ow::FootId ref_foot);

  /*!
  * \brief Find the non reference/suporting foot.
  *
  * \param ref_foot
  *    Reference/suporting foot. This foot is assumed to be in static contact
  *    with the ground.
  *
  * \return
  *    The BodyPartId of the foot which is not the reference/suporting foot.
  */
  ow::FootId findOtherFoot(const ow::FootId ref_foot);

};

}

#endif // OPEN_WALKER_FK_H
