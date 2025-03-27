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

#ifndef OPEN_WALKER_FK_BODY_PART_H
#define OPEN_WALKER_FK_BODY_PART_H

#include <ow_rbdl/rbdl.h>

namespace ow_fk
{

/*!
 * \brief The BodyPart class
 *
 * This class stores the name-id values from the rbdl map to make the forward
 * kinematics more effitiently.
 */
class BodyPart
{
protected:
  std::string name_;    //!< Name of the BodyPart.
  unsigned int id_;     //!< RBDL id of the BodyPart.

public:
  /*!
   * \brief Default constructor.
   *
   * \param name
   *    Name of the BodyPart.
   *
   * \param id
   *    RBDL id of the BodyPart.
   */
  BodyPart(const std::string& name = "", unsigned int id = 0);

  /*!
   * \brief const access to name.
   *
   * \return
   *    Name of the BodyPart.
   */
  const std::string& name() const;

  /*!
   * \brief access to name.
   *
   * \return
   *    Reference to name of the BodyPart.
   */
  std::string& name();

  /*!
   * \brief const access to id.
   *
   * \return
   *    RBDL id.
   */
  const unsigned int& id() const;

  /*!
   * \brief access to id.
   *
   * \return
   *    RBDL id.
   */
  unsigned int & id();

};

}

#endif // OPEN_WALKER_FK_BODY_PART_H
