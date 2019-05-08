/** 
 * @file
 * @brief Defines the state space thermal model for a stepper servo
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

#ifndef _ASC_THERMAL_MODEL_STATE_SPACE_H_
#define _ASC_THERMAL_MODEL_STATE_SPACE_H_

#include "rk4solver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ASC_THERMAL_MODEL_NUM_STATES (3U)
#define ASC_THERMAL_MODEL_NUM_INPUTS (3U)
#define ASC_THERMAL_MODEL_NUM_OUTPUTS (4U)

extern RK4SOLVER_CONFIGURATION * ASC_THERMAL_MODEL_config;

#ifdef __cplusplus
}
#endif

#endif
