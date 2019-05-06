#ifndef _ASC_RK4SOLVER_H_
#define _ASC_RK4SOLVER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        uint32_t numStates;
        uint32_t numInputs;
        uint32_t numOutputs;
        float *A;
        float *B;
        float *C;
        float *D;
    } RK4SOLVER_CONFIGURATION;
    
    typedef struct {
        float h;
        float * currentState;
        float * currentInput;
        float * nextInput;
    } RK4SOLVER_INPUT;
    
    typedef struct {
        float * nextState;
        float * nextOutput;
    } RK4SOLVER_OUTPUT;
    
    extern uint8_t RK4SOLVER_Solve( RK4SOLVER_CONFIGURATION * config,
                                    RK4SOLVER_INPUT * input,
                                    RK4SOLVER_OUTPUT * output );

#ifdef __cplusplus
}
#endif

#endif
