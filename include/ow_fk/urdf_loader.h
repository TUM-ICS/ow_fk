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

#ifndef OPEN_WALKER_FK_URDF_LOADER_H
#define OPEN_WALKER_FK_URDF_LOADER_H

#include <urdf/model.h>

namespace ow_fk
{

/*!
 * \brief The UrdfLoader class
 *
 * This class loads and verifies the URDF model which should be provided as a
 * string or a file. If the URDF model is correct it will be then fit for
 * building the RBDL library model. If not, an error shall return.
 */
class UrdfLoader
{

protected:

  urdf::Model urdf_model_;       //!< URDF model.

public:

  /*!
   * \brief UrdfLoader
   *    Default constructor.
   */
  UrdfLoader();

  /*!
   * \brief Load and verify the robot model from an URDF file.
   *
   * \param urdfFileName
   *    Ful path to the URDF file.
   *
   * \return
   *    true on success.
   */
  bool initFromFile(std::string urdfFileName = "");

  /*!
   * \brief Load and verify the robot model from the ROS parameter server.
   *
   * \param paramName
   *    Name of the ros parameter where the urdf model is loaded.
   *
   * \return
   *    true on success.
   */
  bool initFromParam(std::string paramName = "");

  /*!
   * \brief Get urdf model.
   *
   * \return
   *    Reference to urdf model.
   */
  urdf::Model& urdf();

  std::string urdfString();
};

}

#endif // OPEN_WALKER_FK_URDF_LOADER_H
