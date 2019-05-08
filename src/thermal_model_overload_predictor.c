/** 
 * @file
 * @brief Definition and implementation of the thermal model overload 
 * temperature predictor
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
#include "thermal_model_overload_predictor.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>


bool ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_IsOverloadAvailable( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj )
{
  bool status = false;
  
  if ( obj )
  {
    uint32_t itr = 0U;
    
    for ( itr = 0U; itr < obj->stateSpaceConfig->numOutputs; itr++ )
    {
      if ( obj->maxTemps[ itr ] <= obj->maxTempThresholds[ itr ] )
      {
        status = true;
      }
      else
      {
        status = false;
        break;
      }
    }
  }
  
  return status;
}

void ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_BackgroundTask( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj )
{
  if ( obj )
  {
    float t = obj->solverInputs->h;
    uint32_t itr = 0U;
    
    for ( itr = 0U; itr < obj->periodCounts; itr++ )
    {
      if ( itr < obj->overloadCounts )
      {
        obj->solverInputs->currentInput = (float*)&obj->overloadInputs;
        obj->solverInputs->nextInput = (float*)&obj->overloadInputs;
      }
      else if ( itr == obj->overloadCounts )
      {
        obj->solverInputs->nextInput = (float*)&obj->ratedInputs;
      }
      else
      {
        obj->solverInputs->currentInput = (float*)&obj->ratedInputs;
        obj->solverInputs->nextInput = (float*)&obj->ratedInputs;
      }
      
      if ( RK4SOLVER_Solve( obj->stateSpaceConfig, obj->solverInputs, obj->solverOutputs ) == 1U )
      {
        uint32_t j = 0U;
        
        for ( j = 0U; j < obj->stateSpaceConfig->numOutputs; j++ )
        {
          obj->maxTemps[ j ] = fmaxf( obj->solverOutputs->nextOutput[ j ], obj->maxTemps[ j ] );
        }
        
        t += obj->solverInputs->h;
      }
      else
      {
        break;
      }
    }
  }
}

void ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_UpdateAmbientTemperature( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, float ambient )
{
  if ( obj )
  {
    float difference = obj->ambientTemp - ambient;
    uint32_t itr = 0U;
    
    for ( itr = 0U; itr < obj->stateSpaceConfig->numOutputs; itr++ )
    {
      obj->maxTempThresholds[ itr ] + difference;
    }
  }
}