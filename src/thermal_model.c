/** 
 * @file
 * @brief Definition and implementation of the motor and driver thermal model
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
#include "thermal_model.h"
#include "thermal_model_estimator.h"
#include "thermal_model_overload_predictor.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


static RK4SOLVER_INPUT _overloadPredictorInput;
static RK4SOLVER_OUTPUT _overloadPredictorOutput;
static ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR _overloadPredictor = 
{
  1.0f,
  (uint32_t)(60.0f/1.0f),
  (uint32_t)(10.0f/1.0f),
  20.0f,
  { 0.0f, 0.0f, 0.0f, 0.0f },
  { 80.0f-20.0f, 60.0f-20.0f, 60.0f-20.0f, 80.0f-20.0f },
  { 0.0f, 0.0f, 0.0f },
  { 5.4168f, 23.0400f, 5.5027f },
  { 5.4168f, 16.0000f, 4.4368f },
  (void*)0,
  (void*)0,
  (void*)0
};
static bool _setupOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, RK4SOLVER_INPUT * rk4Input, RK4SOLVER_OUTPUT * rk4Output );
static bool _cleanupOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj );
static void _updateOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, float ambient, float * initialState );

static RK4SOLVER_INPUT _estimatorInput;
static RK4SOLVER_OUTPUT _estimatorOutput;
static ASC_THERMAL_MODEL_ESTIMATOR _estimator = 
{
  0.1f,
  (uint32_t)(1.0f/0.1f),
  20.0f,
  { 0.0f, 0.0f, 0.0f },
  { 0.0f, 0.0f, 0.0f },
  (void*)0,
  (void*)0,
  (void*)0
};

static bool _setupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj, RK4SOLVER_INPUT * rk4Input, RK4SOLVER_OUTPUT * rk4Output );
static bool _cleanupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj );

extern bool ASC_THERMAL_MODEL_Setup( void )
{
  bool status = true;
  
  status &= _setupOverloadPredictor( &_overloadPredictor, &_overloadPredictorInput, &_overloadPredictorOutput );
  status &= _setupEstimator( &_estimator, &_estimatorInput, &_estimatorOutput );
  
  return status;
}

extern bool ASC_THERMAL_MODEL_Cleanup( void )
{
  bool status = true;
  
  status &= _cleanupOverloadPredictor( &_overloadPredictor );
  status &= _cleanupEstimator( &_estimator );
  
  return status;
}

void ASC_THERMAL_MODEL_BackgroundTask( void )
{
  ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_BackgroundTask( &_overloadPredictor );
}

void ASC_THERMAL_MODEL_PeriodicTask( void )
{
  ASC_THERMAL_MODEL_ESTIMATOR_PeriodicTask( &_estimator );
  
  _updateOverloadPredictor( &_overloadPredictor, _estimator.ambientTemp, _estimator.solverOutputs->nextState );
}

bool ASC_THERMAL_MODEL_IsOverloadAvailable( void )
{
  return ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_IsOverloadAvailable( &_overloadPredictor );
}

uint32_t ASC_THERMAL_MODEL_GetCurrentTemp( float * temperatures )
{
  memcpy( (char*)temperatures, 
          (char*)_estimator.solverOutputs->nextOutput, 
          ASC_THERMAL_MODEL_NUM_OUTPUTS * sizeof( float ) );
  return ASC_THERMAL_MODEL_NUM_OUTPUTS;
}

uint32_t ASC_THERMAL_MODEL_GetOLTemp( float * temperatures )
{
  
  memcpy( (char*)temperatures, 
          (char*)_overloadPredictor.maxTemps, 
          ASC_THERMAL_MODEL_NUM_OUTPUTS * sizeof( float ) );
  return ASC_THERMAL_MODEL_NUM_OUTPUTS;
}

void ASC_THERMAL_MODEL_SetInputs( float * inputs )
{
  ASC_THERMAL_MODEL_ESTIMATOR_SetInputs( &_estimator, inputs );
}

static bool _setupOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, RK4SOLVER_INPUT * rk4Input, RK4SOLVER_OUTPUT * rk4Output )
{
  bool status = false;
  
  if ( obj && rk4Input && rk4Output )
  {
    rk4Input->h = obj->h;
    
    rk4Input->currentState = calloc( ASC_THERMAL_MODEL_NUM_STATES, sizeof( float ) );
    memcpy( (char*)rk4Input->currentState,
            (char*)obj->initialState, 
            ASC_THERMAL_MODEL_NUM_STATES * sizeof( float ) );
      
    rk4Output->nextState = rk4Input->currentState;
    rk4Output->nextOutput = calloc( ASC_THERMAL_MODEL_NUM_OUTPUTS, sizeof( float ) );
    
    obj->stateSpaceConfig = ASC_THERMAL_MODEL_config;
    obj->solverInputs = rk4Input;
    obj->solverOutputs = rk4Output;
    
    status = true;
  }
  
  return status;
}

static bool _cleanupOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj )
{
  bool status = false;
  
  if ( obj )
  {
    free( obj->solverInputs->currentState );
    obj->solverInputs->currentState = 0;
    
    free( obj->solverOutputs->nextOutput );
    obj->solverOutputs->nextOutput = 0;
    
    status = true;
  }
  
  return status;
}

static void _updateOverloadPredictor( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR * obj, float ambient, float * initialState )
{
  ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_UpdateAmbientTemperature( obj, ambient );
  memcpy( (char*)obj->solverInputs->currentState,
          (char*)initialState, 
          ASC_THERMAL_MODEL_NUM_STATES * sizeof( float ) );
}

static bool _setupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj, RK4SOLVER_INPUT * rk4Input, RK4SOLVER_OUTPUT * rk4Output )
{
  bool status = false;
  
  if ( obj && rk4Input && rk4Output )
  {
    rk4Input->h = obj->h;
    
    rk4Input->currentState = calloc( ASC_THERMAL_MODEL_NUM_STATES, sizeof( float ) );
    memcpy( (char*)rk4Input->currentState,
            (char*)obj->initialState, 
            ASC_THERMAL_MODEL_NUM_STATES * sizeof( float ) );
    
    rk4Input->currentInput = (float*)obj->aveInputs;
    rk4Input->nextInput = (float*)obj->aveInputs;
    
    rk4Output->nextState = rk4Input->currentState;
    rk4Output->nextOutput = calloc( ASC_THERMAL_MODEL_NUM_OUTPUTS, sizeof( float ) );
    
    obj->stateSpaceConfig = ASC_THERMAL_MODEL_config;
    obj->solverInputs = rk4Input;
    obj->solverOutputs = rk4Output;
    
    status = true;
  }
  
  return status;
}

static bool _cleanupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj )
{
  bool status = false;
  
  if ( obj )
  {
    free( obj->solverInputs->currentState );
    obj->solverInputs->currentState = 0;
    
    free( obj->solverOutputs->nextOutput );
    obj->solverOutputs->nextOutput = 0;
    
    status = true;
  }
    
    return status;
}
