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
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


static RK4SOLVER_INPUT _overloadPredictorInput;
static RK4SOLVER_OUTPUT _overloadPredictorOutput;
static ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR _overloadPredictor = 
{
  1.0f, // sample time
  (uint32_t)(60.0f/1.0f), // thermal period 
  (uint32_t)(10.0f/1.0f), // overload period
  20.0f, // ambient
  { 0.0f, 0.0f, 0.0f, 0.0f }, // maximum observed temps
  { 80.0f-20.0f, 60.0f-20.0f, 60.0f-20.0f, 80.0f-20.0f }, // temperature thresholds (relative to ambient)
  { 0.0f, 0.0f, 0.0f }, // Initial State at start of thermal period t=0
  { 5.4168f, 23.0400f, 5.5027f }, // Overload Maximum Thermal Inputs
  { 5.4168f, 16.0000f, 4.4368f }, // Rated Maximum Thermal Inputs
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
  0.1f, // sample time
  (uint32_t)(1.0f/0.1f), // thermal period
  20.0f, // ambient
  { 0.0f, 0.0f, 0.0f }, // Initial state at start of thermal period t=0
  { 0.0f, 0.0f, 0.0f }, // Actual Thermal Inputs from the thermal period
  (void*)0,
  (void*)0,
  (void*)0
};

static bool _setupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj, RK4SOLVER_INPUT * rk4Input, RK4SOLVER_OUTPUT * rk4Output );
static bool _cleanupEstimator( ASC_THERMAL_MODEL_ESTIMATOR * obj );

/*!
 * \brief Setup and allocation of the overload predictor and estimator
 * \return success
 */
bool ASC_THERMAL_MODEL_Setup( void )
{
  bool status = true;
  
  status &= _setupOverloadPredictor( &_overloadPredictor, &_overloadPredictorInput, &_overloadPredictorOutput );
  status &= _setupEstimator( &_estimator, &_estimatorInput, &_estimatorOutput );
  
  return status;
}

/*!
 * \brief Cleanup and deallocation of the overload predictor and estimator
 * \return success
 */
bool ASC_THERMAL_MODEL_Cleanup( void )
{
  bool status = true;
  
  status &= _cleanupOverloadPredictor( &_overloadPredictor );
  status &= _cleanupEstimator( &_estimator );
  
  return status;
}

/*!
 * \brief A background task that runs the Overload Predictor.
 */
void ASC_THERMAL_MODEL_BackgroundTask( void )
{
  ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_BackgroundTask( &_overloadPredictor );
}

/*!
 * \brief A periodic task that calculates the current temperature based on the
 * previous period
 */
void ASC_THERMAL_MODEL_PeriodicTask( void )
{
  ASC_THERMAL_MODEL_ESTIMATOR_PeriodicTask( &_estimator );
  
  _updateOverloadPredictor( &_overloadPredictor, _estimator.ambientTemp, _estimator.solverOutputs->nextState );
}

/*!
 * \brief Determines if overload capacity is available for the next thermal period
 * \return Overload Capacity Availability
 * \retval true Overload capacity available
 * \retval false Overload Capacity is not available
 */
bool ASC_THERMAL_MODEL_IsOverloadAvailable( void )
{
  return ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_IsOverloadAvailable( &_overloadPredictor );
}

/*! 
 * \brief Gets the current estimated termperatures of the system
 * \param temperatures [out] Array of system temperatures
 * \return Number of temperatures in output parameter
 */
uint32_t ASC_THERMAL_MODEL_GetCurrentTemp( float * temperatures )
{
  memcpy( (char*)temperatures, 
          (char*)_estimator.solverOutputs->nextOutput, 
          ASC_THERMAL_MODEL_NUM_OUTPUTS * sizeof( float ) );
  return ASC_THERMAL_MODEL_NUM_OUTPUTS;
}

/*!
 * \brief Gets the overload maximum system temperatures for the next thermal 
 * period
 * \param temperatures [out] Array of system temperatures
 * \return Number of temperature sin output parameter
 */
uint32_t ASC_THERMAL_MODEL_GetOLTemp( float * temperatures )
{
  memcpy( (char*)temperatures, 
          (char*)_overloadPredictor.maxTemps, 
          ASC_THERMAL_MODEL_NUM_OUTPUTS * sizeof( float ) );
  return ASC_THERMAL_MODEL_NUM_OUTPUTS;
}

/*!
 * \brief Sets the thermal source inputs used for the thermal estimator
 * \param inputs Array of thermal heat source inputs for the previous period
 */
void ASC_THERMAL_MODEL_SetInputs( float * inputs )
{
  ASC_THERMAL_MODEL_ESTIMATOR_SetInputs( &_estimator, inputs );
}

/*!
 * \brief Calculates the thermal inputs based on drive current and rotational speed
 * \param sourceInputs [out] The calculated thermal inputs in Watts
 * \param driveCurrent The drive current applied to the system in Amps
 * \param rotationalSpeed The rotational speed of the motor in rad/s
 */
void ASC_THERMAL_MODEL_CalculateSourceInputs( float * sourceInputs, float driveCurrent, float rotationalSpeed )
{
  if ( sourceInputs )
  {
    float phaseResistancex2 = 2.0f * 1.0f;
    float rdsOnx4 = 4.0f * 1.325E-02f;
    float busVoltagex4xtRiseFallxfSwitching = 4.0f * 48.0f * 1.4E+05f * (15E-09f + 19E-09f);
    float rsnsx2 = 2.0f * 2.0E-02f;
    float oneOverSqrt2 = 0.70711f;
    float driveCurrentRms = driveCurrent * oneOverSqrt2;
    float driveCurrentRmsSquared = driveCurrent * driveCurrent / 2.0f;
    float measuredOtherPowerComonents = 0.27f;
    
    sourceInputs[ 0U ] = 3.03e-02f * powf( rotationalSpeed, 1.44f );
    sourceInputs[ 1U ] = phaseResistancex2 * driveCurrentRmsSquared;
    sourceInputs[ 2U ] = rdsOnx4 * driveCurrentRmsSquared +
                         busVoltagex4xtRiseFallxfSwitching * driveCurrentRms +
                         rsnsx2 * driveCurrentRmsSquared + 
                         measuredOtherPowerComonents;
  }
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
