/** 
 * @file
 * @brief Definition and implementation of a State Space Runge-Kutta 4 ODE solver 
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
#include <float.h>
#include <stdint.h>

static const float ONEBYSIX = (1.0f/6.0f);

/*!
 * \brief Matrix get accessor
 * \param matrix Pointer to RAM containing 2D array
 * \param numColumns number of columns to parse 2D array
 * \param row The row to access
 * \param column The column to access
 * \return pointer to specified element
 * \note As this is a static funtion, there is no input validation
 */
static float * _Get( float * matrix,
                     uint32_t numColumns,
                     uint32_t row, 
                     uint32_t column )
{
  float * element = matrix + ( row * numColumns ) + column;
  
  return element;
}

/*!
 * \brief Adds two 1-D arrays: result = lhs + rhs
 * \param lhs First operand, pointer to Array
 * \param rhs Second operand, pointer to Array
 * \param result [out] Result of the add operation, pointer to Array
 * \param numElements Length of the lhs and rhs arrays
 * \note As this is a static function, there is no input validation
 */
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

/*!
 * \brief Adds four arrays result = arry1 + arry2 + arry3 + arry4
 * \param arry1 First operand, pointer to Array
 * \param arry2 Second operand, pointer to Array
 * \param arry3 Third operand, pointer to Array
 * \param arry4 Fourth operand, pointer to Array
 * \param result [out] Result of add operation, pointer to Array
 * \param numElements Length of the operand arrays
 * \note As this is a static function, there is no input validation
 */
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

/*!
 * \brief Element-wise array multiply by a scaler [result] = [lhs] * rhs
 * \param lhs Array to be scaled, pointer to Array
 * \param rhs Scalar multiplier
 * \param [out] result Result of the multiply operation, pointer to Array
 * \param numElements Length of first operand array
 * \note As this is a static function, there is no input validation
 */
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

/*!
 * \brief Copy of src array to dst array dst = src
 * \param dst [out] Pointer to destination array
 * \param src Pointer to source array
 * \param numElements Length of array to be copied
 * \note As this is a static function, there is no input validation
 */
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

/*!
 * \brief Calculates xdot result for given state (x) and input (u) such that
 * [result] = [A]*x + [B]*u
 * \param config The configuation structure containing A, B and numStates and numInputs
 * \param x Pointer to the states array used in calculation
 * \param u Pointer to the inputs array used in calculatoin
 * \param result [out] Pointer to xdot result array
 * \note As this is a static function, there is no input validation
 */
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
      float A = *_Get( config->A, config->numStates, i, j );
      
      if ( A != 0.0 )
      {
        result[ i ] += A * x[ j ];
      }
    }
    
    for( j = 0U; j < config->numInputs; j++ )
    {
      float B = *_Get( config->B, config->numInputs, i, j );
      
      if ( B != 0.0 )
      {
        result[ i ] += B * u[ j ];
      }
    }
  }
}

/*!
 * \brief Calculates the output y for the given states and inputs such that
 * [yn+1] = [C]*x + [D]*u
 * \param config The configuration structure containing C, D, numStates, numInputs, numOutputs
 * \param input The input structure containing xn, un, un+1
 * \param output [out] The output structure containing yn+1
 * \note As this is a static function, there is no input validation
 */
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

/*!
 * \brief Calculates the next state and output of the state space 
 * representation using Runge-Kutta 4 numeric method to solve ODEs:
 *  fx( x, u ) = [A]*x + [B]*u
 *  [xn+1] = [x] + 1/6 ([K0] + 2.*[K1] + 2.*[K2] + K3)
 *    where:
        K0 = fx( xn, un )
        K1 = fx( xn + h/2.*K0, 1/2.*(un+un+1) )
        K2 = fx( xn + h/2.*K1, 1/2.*(un+un+1) )
        K3 = fx( xn + h.*K2, un+1 )
 *  [y] = [C]*x + [B]*u
 * \returns success of failure and fill in output if successful
 * \retval 0U Failure
 * \retval 1U Success
 * \note All vectors must be of stated length and pointers are non-null
 */ 
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
