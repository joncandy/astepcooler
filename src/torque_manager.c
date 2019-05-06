/** 
 * @file
 * @brief Definition and implementation of the stepper motor torque manager
 * @author Jon C. Anderson <andersonjc@msoe.edu>
 * @copyright (C) Jon C. Anderson 2019
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "torque_manager.h"
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/*!
 * \brief Applies and upper, saturating limit
 * \param value The value being limited
 * \param limit The limit being applied
 * \return limited value 
 * \private 
 */
static uint8_t _applyLimit( uint8_t value, uint8_t limit )
{
  uint8_t retVal = value;
  
  if( value > limit )
  {
    retVal = limit;
  }
  
  return retVal;    
}

/*!
 * \brief Sets the torque value by enumerated index
 * \param obj The Torque Manager Instance being modified
 * \param index The enumerated index used to select the setpoint from the setpoint list
 * \note Applies setpoint limit to result
 * \return Resulting torque value 
 */
uint8_t ASC_TORQUE_MANAGER_SetTorqueByIndex( ASC_TORQUE_MANAGER * obj, uint8_t index )
{
  uint8_t retVal = 0U;
  
  if( obj )
  {
    if( index < ASC_TORQUE_SETPOINT_COUNT )
    {
      obj->activeSetpointValue = _applyLimit( obj->setpoints[ index ], obj->setpointLimit );
      obj->activeSetpointIndex = index;
    }
    
    retVal = obj->activeSetpointValue;
  }
  
  return retVal;
}

/*!
 * \brief Sets the setpoint limit applied before
 * \param obj The Torque Manager Instance being modified
 * \param index The enumerated index used to select the setpoint from the setpoint list
 * \note Applies setpoint limit to activeSetpointValue
 * \return Resulting torque value with limit applied 
 */
uint8_t ASC_TORQUE_MANAGER_SetSetpointLimit( ASC_TORQUE_MANAGER * obj, uint8_t limit )
{
  uint8_t retVal = 0U;
  
  if( obj )
  {
    obj->setpointLimit = limit;
    obj->activeSetpointValue = _applyLimit( obj->activeSetpointValue, obj->setpointLimit );
    
    retVal = obj->activeSetpointValue;
  }
  
  return retVal;
}

/*!
 * \brief Sets the feedforward value that is added to the setpoint
 * \param obj The Torque Manager Instance being modified
 * \param feedforward The value to be applied
 * \return Resulting feedforward value 
 */
uint8_t ASC_TORQUE_MANAGER_SetFeedforwardValue( ASC_TORQUE_MANAGER * obj, uint8_t feedforward )
{
  uint8_t retVal = 0U;
  
  if( obj )
  {
    obj->activeFeedforwardValue = feedforward;
    retVal = obj->activeFeedforwardValue;
  }
  
  return retVal;
}

/*!
 * \brief A foreground task responsible for applying limited torque value and 
 *  feedforward value to the system via the setTorque function.
 * \note Updates lastSetpointValue and lastFeedforwardValue
 * \param obj The Torque Manager Instance being modified
 */
void ASC_TORQUE_MANAGER_ForegroundTask( ASC_TORQUE_MANAGER * obj )
{
  if( obj )
  {
    uint8_t changeNeeded = ( obj->lastSetpointValue != obj->activeSetpointValue ) ||
                           ( obj->lastFeedforwardValue != obj->activeFeedforwardValue );
                           
    if( obj->setTorque && changeNeeded )
    {
      uint8_t limitedSetpoint = _applyLimit( obj->activeSetpointValue + obj->activeFeedforwardValue, obj->setpointLimit );
      (*obj->setTorque)( limitedSetpoint );
      obj->lastSetpointValue = obj->activeSetpointValue;
      obj->lastFeedforwardValue = obj->activeFeedforwardValue;
    }
  }
}
