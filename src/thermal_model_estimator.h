/** 
 * @file
 * @brief Defines the interface to the thermal model temperature estimator
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
#ifndef _ASC_THERMAL_MODEL_ESTIMATOR_H_
#define _ASC_THERMAL_MODEL_ESTIMATOR_H_

#include "rk4solver.h"
#include "thermal_model_state_space.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  /*!
   * \brief A structure containing the Thermal Estimator model parameters
   */
  typedef struct 
  {
      float h; //!< time step
      uint32_t periodCounts; //!< number of time steps in the thermal period
      float ambientTemp; //!< The ambient temperature
      float initialState[ ASC_THERMAL_MODEL_NUM_STATES ]; //!< The initial temperatures for the thermal period
      float aveInputs[ ASC_THERMAL_MODEL_NUM_INPUTS ]; //!< The heat source inputs from the period
      RK4SOLVER_CONFIGURATION * stateSpaceConfig; //!< The state space thermal model
      RK4SOLVER_INPUT * solverInputs; //!< Collection of thermal inputs for the RK4 Solver
      RK4SOLVER_OUTPUT * solverOutputs; //!< Collection of thermal outputs for the RK4 Solver
  } ASC_THERMAL_MODEL_ESTIMATOR;
  
  extern void ASC_THERMAL_MODEL_ESTIMATOR_PeriodicTask( ASC_THERMAL_MODEL_ESTIMATOR * obj );
  extern void ASC_THERMAL_MODEL_ESTIMATOR_SetInputs( ASC_THERMAL_MODEL_ESTIMATOR * obj, float * inputs );
  
#ifdef __cplusplus
}
#endif

#endif
