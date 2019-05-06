#include "rk4solver.h"
#include <float.h>
#include <stdint.h>

static const float ONEBYSIX = (1.0f/6.0f);

static float * _Get( float * matrix,
                     uint32_t numColumns,
                     uint32_t row, 
                     uint32_t column )
{
  float * element = matrix + ( row * numColumns ) + column;
  
  return element;
}

static void _AddArray( float * lhs,
                       float * rhs,
                       float * result,
                       uint32_t numElements )
{
  uint32_t i = 0U;
  
  for ( i = 0U; i < numElements; i++ )
  {
    result[ i ] = lhs[ i ] + rhs[ i ];
  }
}

static void _AddArray4( float * arry1,
                        float * arry2,
                        float * arry3,
                        float * arry4,
                        float * result,
                        uint32_t numElements )
{
  uint32_t i = 0U;
  
  for ( i = 0U; i < numElements; i++ )
  {
    result[ i ] = arry1[ i ] + arry2[ i ] + arry3[ i ] + arry4[ i ];
  }
}
       
static void _DotMultiplyArray( float * lhs,
                               float rhs,
                               float * result,
                               uint32_t numElements )
{
  uint32_t i = 0U;
  
  for ( i = 0U; i < numElements; i++ )
  {
    result[ i ] = lhs[ i ] * rhs;
  }
}

static void _CopyArray( float * dst,
                        float * src,
                        uint32_t numElements )
{
  uint32_t i = 0U;
  
  for ( i = 0U; i < numElements; i++ )
  {
    *dst = *src;
  }
}

static void _fx( RK4SOLVER_CONFIGURATION * config,
                 float * x,
                 float * u,
                 float * result )
{
  uint32_t i = 0U;
  uint32_t j = 0U;
  
  for ( i = 0U; i < config->numStates; i++ )
  {
    result[ i ] = 0.0;
    for ( j = 0U; j < config->numStates; j++ )
    {
      result[ i ] += *_Get( config->A, config->numStates, i, j ) * x[ j ];
    }
    
    for( j = 0U; j < config->numInputs; j++ )
    {
      result[ i ] += *_Get( config->B, config->numInputs, i, j ) * u[ j ];
    }
  }
}

static void _GenerateOutput( RK4SOLVER_CONFIGURATION * config,
                             RK4SOLVER_INPUT * input,
                             RK4SOLVER_OUTPUT * output )
{
  uint32_t i = 0U;
  uint32_t j = 0U;
  
  for ( i = 0U; i < config->numOutputs; i++ )
  {
    output->nextOutput[ i ] = 0.0;
    for ( j = 0U; j < config->numStates; j++ )
    {
      float C = *_Get( config->C, config->numStates, i, j );
      
      if ( C == 1.0f )
      {
        output->nextOutput[ i ] += output->nextState[ j ];
      }
      else if ( C != 0.0f )
      {
        output->nextOutput[ i ] += C * output->nextState[ j ];
      }
    }
    
    for ( j = 0U; j < config->numInputs; j++ )
    {
      float D = *_Get( config->D, config->numInputs, i, j );
      
      if ( D == 1.0f )
      {
        output->nextOutput[ i ] += input->currentInput[ j ];
      }
      else if ( D != 0.0f )
      {
        output->nextOutput[ i ] += D * input->currentInput[ j ];
      }
    }
  }
}

uint8_t RK4SOLVER_Solve( RK4SOLVER_CONFIGURATION * config,
                         RK4SOLVER_INPUT * input,
                         RK4SOLVER_OUTPUT * output )
{
  uint8_t status = 0U; // failure
  
  if ( config && input && output )
  {
    float x[ config->numStates ];
    float u[ config->numInputs ];
    float K[ 4U ][ config->numStates ];
    
    _fx( config, input->currentState , input->currentInput, (float*)&K[ 0 ] );
    
    // x = h/2 .* K[0] + currentState
    // u = 1/2 .* (currentInput + nextInput)
    _DotMultiplyArray( (float*)&K[ 0 ], input->h * 0.5, x, config->numStates );
    _AddArray( input->currentState, x, x, config->numStates );
    _AddArray( input->currentInput, input->nextInput, u, config->numStates );
    _DotMultiplyArray( u, 0.5, u, config->numInputs );
    _fx( config, x, u, (float*)&K[ 1 ] );
    _DotMultiplyArray( (float*)&K[ 1 ], 2.0, (float*)&K[ 1 ], config->numStates );
    
    // x = h/2 .* K[1] + currentState
    // u = 1/2 .* (currentInput + nextInput)
    _DotMultiplyArray( (float*)&K[ 1 ], input->h * 0.5, x, config->numStates );
    _AddArray( input->currentState, x, x, config->numStates );
    // already done : 
    // _AddArray( input->currentInput, input->nextInput, u, config->numStates );
    // _DotMultiplyArray( u, 0.5, u, config->numInputs );
    _fx( config, x, u, (float*)&K[ 2 ] );
    _DotMultiplyArray( (float*)&K[ 2 ], 2.0, (float*)&K[ 2 ], config->numStates );
    
    // x = h .* K[2] + currentState
    // u = nextInput
    _DotMultiplyArray( (float*)&K[ 2 ], input->h, x, config->numInputs );
    _AddArray( input->currentState, x, x, config->numStates );
    _CopyArray( u, input->nextInput, config->numInputs );
    _fx( config, x, u, (float*)&K[ 3 ] );
    
    // K[0] = K[total] = h/6 ( K[0] + 2.*K[1] + 2.*K[2] + K[3] )
    _AddArray4( (float*)&K[ 0 ],
                (float*)&K[ 1 ], 
                (float*)&K[ 2 ],
                (float*)&K[ 3 ],
                (float*)&K[ 0 ],
                config->numStates );
    _DotMultiplyArray( (float*)&K[ 0 ], input->h * ONEBYSIX, (float*)&K[ 0 ], config->numInputs );
    
    // nextState = currentState + K[total]
    _AddArray( input->currentState, (float*)&K[ 0 ], output->nextState, config->numStates );
    
    _GenerateOutput( config, input, output );
    
    status = 1U; // success
  }
  
  return status;
}