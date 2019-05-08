/** 
 * @file
 * @brief Defines the interface to the thermal model overload temperature 
 * predictor
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
 
#ifndef _ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_H_
#define _ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_H_

#include "rk4solver.h"
#include "thermal_model_state_space.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct 
    {
        float h;
        uint32_t periodCounts;
        uint32_t overloadCounts;
        float ambientTemp;
        float maxTemps[ ASC_THERMAL_MODEL_NUM_OUTPUTS ];
        float maxTempThresholds[ ASC_THERMAL_MODEL_NUM_OUTPUTS ];
        float initialState[ ASC_THERMAL_MODEL_NUM_STATES ];
        float overloadInputs[ ASC_THERMAL_MODEL_NUM_INPUTS ];
        float ratedInputs[ ASC_THERMAL_MODEL_NUM_INPUTS ];
        RK4SOLVER_CONFIGURATION * stateSpaceConfig;
        RK4SOLVER_INPUT * solverInputs;
        RK4SOLVER_OUTPUT * solverOutputs;
    } ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR;

    extern bool ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_IsOverloadAvailable( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj );
    extern void ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_BackgroundTask( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj );
    extern void ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_UpdateAmbientTemperature( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, float ambient );

#ifdef __cplusplus
}
#endif

#endif
