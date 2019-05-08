/** 
 * @file
 * @brief Defines the interface to a State Space Runge-Kutta 4 ODE solver
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

#ifndef _ASC_RK4SOLVER_H_
#define _ASC_RK4SOLVER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    /*! 
     * Defines State Space Representation
     * [dx/dt] = [A]*x + [B]*u
     * [y] = [C]*x + [D]*u
     */
    typedef struct {
        uint32_t numStates; //!< Row and Columns for A, Rows for B
        uint32_t numInputs; //!< Columns for B and D
        uint32_t numOutputs; //!< Rows and Columns for C, Rows for D
        float *A;
        float *B;
        float *C;
        float *D;
    } RK4SOLVER_CONFIGURATION;

    /*! 
     * Defines current iteration State (xn) and Input (un) as well as 
     * next iteration Input (un+1).
     * Used in fx( x, u )
     * \note In this implementation either x or u can be time variant 
     * but must be calculated outside of this component.
     */
    typedef struct {
        float h; //!< time step
        float * currentState; //!< xn input must be numStates long
        float * currentInput; //!< un input must be numInputs long
        float * nextInput; //!< un+1 input must be numInputs long
    } RK4SOLVER_INPUT;
    
    /*! 
     * Defines the outputs xn+1, yn+1
     */
    typedef struct {
        float * nextState; //!< xn+1 output must be numStates long
        float * nextOutput; //!< yn+1 output mus be numOutputs long
    } RK4SOLVER_OUTPUT;
    
    extern uint8_t RK4SOLVER_Solve( RK4SOLVER_CONFIGURATION * config,
                                    RK4SOLVER_INPUT * input,
                                    RK4SOLVER_OUTPUT * output );

#ifdef __cplusplus
}
#endif

#endif
