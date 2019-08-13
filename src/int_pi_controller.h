/** 
 * @file
 * @brief Defines the interface and implementation of a PID controller
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

#ifndef _ASC_INT_PI_CONTROLLER_H
#define _ASC_INT_PI_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
    typedef struct
    {
        int32_t kp_num;
        int32_t kp_div;
        int32_t ki_num;
        int32_t ki_div;
        int32_t iSum;
        uint8_t iSumMax;
        uint8_t iSumMin;
        uint8_t reset;
    } INT_8_PI_CONTROLLER_t;
    
extern uint8_t PI_Step( INT_8_PI_CONTROLLER_t * controller, uint8_t setpoint, int32_t feedback, uint8_t feedforward );
void PI_Reset( INT_8_PI_CONTROLLER_t * controller );

#ifdef __cplusplus
}
#endif

#endif



    