#include "astepcooler_test.h"
#include "rk4solver.h"
#include "thermal_model.h"
#include "thermal_model_overload_predictor.h"
#include "thermal_model_state_space.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RUN_THERMAL_MANAGER true
#define PRINT_TEMPERATURES false

RK4SOLVER_INPUT rk4input;
RK4SOLVER_OUTPUT rk4output;

ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR overloadPredictor = 
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

static void _setupRK4Solver( RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output );
static void _cleanupRK4Solver( RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output );

int main( int argc, char *argv[] )
{
  if ( RUN_THERMAL_MANAGER )
  {
    uint32_t itr = 0U;
    float ins[ ASC_THERMAL_MODEL_NUM_INPUTS ];
    float temp[ ASC_THERMAL_MODEL_NUM_OUTPUTS ];
    ASC_THERMAL_MODEL_Setup();
    
    ASC_THERMAL_MODEL_CalculateSourceInputs( (float*)ins, 4.0f, 36.652 );
    
    printf( "inputs: [ %6.4f, %6.4f, %6.4f ]\n", ins[ 0U ], ins[ 1U ], ins[ 2U ] );
    
    for ( itr = 0U; itr <= 3600; itr++ )
    {
    
      ASC_THERMAL_MODEL_SetInputs( ins );
      ASC_THERMAL_MODEL_PeriodicTask();
      ASC_THERMAL_MODEL_BackgroundTask();
      
      printf( "%4u ", itr );
      
      ASC_THERMAL_MODEL_GetCurrentTemp( (float*)temp );
      printf( "%7.4f %7.4f %7.4f %7.4f | ", temp[ 0U ], temp[ 1U ], temp[ 2U ], temp[ 3U ] );
      
      ASC_THERMAL_MODEL_GetOLTemp( (float*)temp );
      printf( "%7.4f %7.4f %7.4f %7.4f | ", temp[ 0U ], temp[ 1U ], temp[ 2U ], temp[ 3U ] );
      
      printf( "OL Allowed: " );
      if ( ASC_THERMAL_MODEL_IsOverloadAvailable() )
      {
        printf( "yes\n" );
      }
      else
      {
        printf( "no\n" );
      }
    }
    
    ASC_THERMAL_MODEL_Cleanup();
  }
  
  if ( PRINT_TEMPERATURES )
  {
    float t = 0.0f;
    uint32_t itr = 0U;
    
    _setupRK4Solver( &rk4input, &rk4output );
    overloadPredictor.stateSpaceConfig = ASC_THERMAL_MODEL_config;
    overloadPredictor.solverInputs = &rk4input;
    overloadPredictor.solverOutputs = &rk4output;
    
    ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_BackgroundTask( &overloadPredictor );
    
    t = overloadPredictor.solverInputs->h;
  
    printf( "# %s V %d.%d.%d\n", argv[0], VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH );
    printf( "# t   s1      s2      s3     s4\n" );
    
    for ( itr = 0U; itr < overloadPredictor.periodCounts; itr++ )
    {
      if ( itr < overloadPredictor.overloadCounts )
      {
        overloadPredictor.solverInputs->currentInput = (float*)&overloadPredictor.overloadInputs;
        overloadPredictor.solverInputs->nextInput = (float*)&overloadPredictor.overloadInputs;
      }
      else if ( itr == overloadPredictor.overloadCounts )
      {
        overloadPredictor.solverInputs->nextInput = (float*)&overloadPredictor.ratedInputs;
      }
      else
      {
        overloadPredictor.solverInputs->currentInput = (float*)&overloadPredictor.ratedInputs;
        overloadPredictor.solverInputs->nextInput = (float*)&overloadPredictor.ratedInputs;
      }
      
      if ( RK4SOLVER_Solve( overloadPredictor.stateSpaceConfig, overloadPredictor.solverInputs, overloadPredictor.solverOutputs ) == 1U )
      {
        uint32_t j = 0U;
        
        printf( "%4.1f ", t );
        
        for ( j = 0U; j < overloadPredictor.stateSpaceConfig->numOutputs; j++ )
        {
          printf( "%7.4f ", overloadPredictor.solverOutputs->nextOutput[ j ] );
          overloadPredictor.maxTemps[ j ] = fmaxf( overloadPredictor.solverOutputs->nextOutput[ j ], overloadPredictor.maxTemps[ j ] );
        }
        
        printf( "\n" );
                                                      
        t += overloadPredictor.solverInputs->h;
      }
      else
      {
        printf( "didn't work\n" );
        break;
      }
    }
  
    printf( "MAX  " );
    
    for ( itr = 0U; itr < overloadPredictor.stateSpaceConfig->numOutputs; itr++ )
    {
      printf( "%7.4f ", overloadPredictor.maxTemps[ itr ] );
    }
    
    printf( "\nLim  " );
    
    for ( itr = 0U; itr < overloadPredictor.stateSpaceConfig->numOutputs; itr++ )
    {
      printf( "%7.4f ", overloadPredictor.maxTempThresholds[ itr ] );
    }
    
    printf( "\n" );
    
    printf( "Overload Allowed: " );
    if ( ASC_THERMAL_MODEL_OVERLOAD_PREDICTOR_IsOverloadAvailable( &overloadPredictor ) )
    {
      printf( "yes\n" );
    }
    else
    {
      printf( "no\n" );
    }
    
    _cleanupRK4Solver( &rk4input, &rk4output );
    overloadPredictor.solverInputs = (void*) 0;
    overloadPredictor.solverOutputs = (void*) 0;
  }
  
  
  
  return 0;
}

void _setupRK4Solver( RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output )
{
  input->h = overloadPredictor.h;
  
  input->currentState = calloc( ASC_THERMAL_MODEL_NUM_STATES, sizeof( float ) );
  memcpy( (char*)input->currentState, (char*)overloadPredictor.initialState, ASC_THERMAL_MODEL_NUM_STATES * sizeof( float ) );
    
  output->nextState = input->currentState;
  output->nextOutput = calloc( ASC_THERMAL_MODEL_NUM_OUTPUTS, sizeof( float ) );
}

void _cleanupRK4Solver( RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output )
{
  free( input->currentState );
  input->currentState = 0;
  
  free( output->nextOutput );
  output->nextOutput = 0;
}
