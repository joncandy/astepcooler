/** 
 * @file
 * @brief Definition and implementation of the thermal model temperature 
 * estimator
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
 
#include "rk4solver.h"
#include "thermal_model_estimator.h"
#include <stdint.h>
#include <string.h>

/*!
 * \brief A task intended to be run at the course thermal manager period (1s)
 * to calculate the current system temperatures based on the inputs of the last
 * period.
 * \param obj A pointer to the Thermal Model Estimator data structure
 */
void ASC_THERMAL_MODEL_ESTIMATOR_PeriodicTask( ASC_THERMAL_MODEL_ESTIMATOR * obj )
{
  if ( obj )
  {
    uint32_t itr = 0U;
    
    for ( itr = 0U; itr < obj->periodCounts; itr++ )
    {
      uint32_t result = RK4SOLVER_Solve( obj->stateSpaceConfig,
                                         obj->solverInputs, 
                                         obj->solverOutputs );
      
      if ( result != 1U ) //error condition
      {
        break;
      }
    }
  }
}

/*!
 * \brief Sets the thermal model inputs based on the provided temperature state
 * inputs.
 * \param ojb A pointer to the Thermal Model Estimator data structure
 * \param inputs An array of thermal state inputs
 */
void ASC_THERMAL_MODEL_ESTIMATOR_SetInputs( ASC_THERMAL_MODEL_ESTIMATOR * obj, float * inputs )
{
  if ( obj && inputs )
  {
    memcpy( (char*)obj->aveInputs,
    (char*)inputs,
    obj->stateSpaceConfig->numInputs * sizeof( float ) );
  }
}