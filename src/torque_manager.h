/** 
 * @file
 * @brief Defines the interface to the stepper motor torque manager
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

#ifndef _ASC_TORQUE_MANAGER_H
#define _ASC_TORQUE_MANAGER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    /* Indexes for SetTorqueByIndex selector */
    const uint8_t ASC_TORQUE_OFF_INDEX = 0U;
    const uint8_t ASC_TORQUE_IDLE_INDEX = 1U;
    const uint8_t ASC_TORQUE_ACCEL_PLUS_INDEX = 2U;
    const uint8_t ASC_TORQUE_ACCEL_MINUS_INDEX = 3U;
    const uint8_t ASC_TORQUE_CRUISE_INDEX = 4U;
    const uint8_t ASC_TORQUE_DECEL_PLUS_INDEX = 5U;
    const uint8_t ASC_TORQUE_DECEL_MINUS_INDEX = 6U;
    const uint8_t ASC_TORQUE_FULL_INDEX = 7U;
    #define ASC_TORQUE_SETPOINT_COUNT 8U
    
    typedef struct
    {
        uint8_t setpointLimit;
        uint8_t activeSetpointIndex;
        uint8_t activeSetpointValue;
        uint8_t lastSetpointValue;
        uint8_t activeFeedforwardValue;
        uint8_t lastFeedforwardValue;
        void (*setTorque)(uint8_t value);
        uint8_t setpoints[ ASC_TORQUE_SETPOINT_COUNT ];
    } ASC_TORQUE_MANAGER;
    
    extern uint8_t ASC_TORQUE_MANAGER_SetTorqueByIndex( ASC_TORQUE_MANAGER * obj, uint8_t index );
    extern uint8_t ASC_TORQUE_MANAGER_SetSetpointLimit( ASC_TORQUE_MANAGER * obj, uint8_t limit );
    extern uint8_t ASC_TORQUE_MANAGER_SetFeedforwardValue( ASC_TORQUE_MANAGER * obj, uint8_t feedforward );
    extern void ASC_TORQUE_MANAGER_ForegroundTask( ASC_TORQUE_MANAGER * obj );
    
#ifdef __cplusplus
}
#endif

#endif
