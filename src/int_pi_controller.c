#include "int_pi_controller.h"
#include <stdint.h>

static uint32_t _abs( int32_t i )
{
    return (i < 0) ? i * (-1) : i;
}

static int32_t _sign( int32_t i )
{
    return (i < 0) ? -1 : 1;
}

static int8_t _limit( int32_t i )
{
    if ( i > 0xFF )
    {
        i = 0xFF;
    }
    else if ( i < 0x00 )
    {
        i = 0x00;
    }
    
    return (uint8_t)i;
}

uint8_t PI_Step( INT_8_PI_CONTROLLER_t * controller, uint8_t setpoint, int32_t feedback, uint8_t feedforward )
{
    int32_t error = setpoint - feedback;
    int32_t retVal = feedforward;
    if( controller )
    {
        if( controller->reset != 0U )
        {
            controller->iSum = 0U;
            controller->reset = 0U;
        }
        
        // update and limit integral term 
        controller->iSum += error;
        if( _abs(controller->iSum) > controller->iSumMax )
        {
            controller->iSum = _sign(controller->iSum) * controller->iSumMax;
        }
        else if( _abs(controller->iSum) < controller->iSumMin )
        {
            controller->iSum = _sign(controller->iSum) * controller->iSumMin;
        }
        
        retVal += (controller->kp_num * error) / controller->kp_div;
        retVal += (controller->ki_num * controller->iSum) / controller->ki_div;
    }
            
    return _limit( retVal );
};

void PI_Reset( INT_8_PI_CONTROLLER_t * controller )
{
    if( controller )
    {
        controller->reset = 1U;
    }  
};
