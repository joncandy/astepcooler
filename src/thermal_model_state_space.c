/** 
 * @file
 * @brief Definition of the state space thermal model for a stepper servo
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

#include "thermal_model_state_space.h"
#include "rk4solver.h"
#include <stdint.h>

static float _A[ ASC_THERMAL_MODEL_NUM_STATES ][ ASC_THERMAL_MODEL_NUM_STATES ] = 
    {{-1.5603E-02,  1.4710E-02,  3.3201E-04},
     { 0.0000E+00, -8.9398E-04,  3.3201E-04},
     { 0.0000E+00,  1.0531E-03, -2.6055E-03}};
     
static float _B[ ASC_THERMAL_MODEL_NUM_STATES ][ ASC_THERMAL_MODEL_NUM_INPUTS ] = 
    {{ 3.2095E-02,  9.4706E-03,  0.0000E+00},
     { 1.6690E-03,  1.6690E-03,  0.0000E+00},
     { 0.0000E+00,  0.0000E+00,  5.2938E-03}};
     
static float _C[ ASC_THERMAL_MODEL_NUM_OUTPUTS ][ ASC_THERMAL_MODEL_NUM_STATES ] = 
    {{1, 0, 0},
     {0, 1, 0},
     {0, 0, 1},
     {0, 0, 1}};;
     
static float _D[ ASC_THERMAL_MODEL_NUM_OUTPUTS ][ ASC_THERMAL_MODEL_NUM_INPUTS ] = 
    {{0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 7.475E+00}};

static RK4SOLVER_CONFIGURATION _config =
    { (uint32_t)ASC_THERMAL_MODEL_NUM_STATES,
      (uint32_t)ASC_THERMAL_MODEL_NUM_INPUTS,
      (uint32_t)ASC_THERMAL_MODEL_NUM_OUTPUTS,
      (float*)_A,
      (float*)_B,
      (float*)_C,
      (float*)_D
    };
    
RK4SOLVER_CONFIGURATION * ASC_THERMAL_MODEL_config = &_config;
