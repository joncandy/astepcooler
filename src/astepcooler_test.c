#include "astepcooler_test.h"
#include "rk4solver.h"

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

RK4SOLVER_CONFIGURATION rk4config;
RK4SOLVER_INPUT rk4input;
RK4SOLVER_OUTPUT rk4output;

#define NUM_STATES 3U
#define NUM_INPUTS 3U
#define NUM_OUTPUTS 4U

static float A[ NUM_STATES ][ NUM_STATES ] = 
    {{-1.5603E-02,  1.4710E-02,  3.3201E-04},
     { 0.0000E+00, -8.9398E-04,  3.3201E-04},
     { 0.0000E+00,  1.0531E-03, -2.6055E-03}};
static float B[ NUM_STATES ][ NUM_INPUTS ] = 
    {{ 3.2095E-02,  9.4706E-03,  0.0000E+00},
     { 1.6690E-03,  1.6690E-03,  0.0000E+00},
     { 0.0000E+00,  0.0000E+00,  5.2938E-03}};
static float C[ NUM_OUTPUTS ][ NUM_STATES ] = 
    {{1, 0, 0},
     {0, 1, 0},
     {0, 0, 1},
     {0, 0, 1}};;
static float D[ NUM_OUTPUTS ][ NUM_INPUTS ] = 
    {{0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 0.000E+00},
     {0.000E+00, 0.000E+00, 7.475E+00}};

static float CurrentState[ NUM_STATES ] = { 0, 0, 0 };
static float CurrentInput[ NUM_INPUTS ] = { 5.4168, 16.0000, 4.4368 };
static float NextInput[ NUM_INPUTS ] = { 5.4168, 16.0000, 4.4368 };
        
static float NextState[ NUM_STATES ];
static float NextOutput[ NUM_OUTPUTS ];

static void _setupRK4Solver( RK4SOLVER_CONFIGURATION * config, RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output );

int main( int argc, char *argv[] )
{
  float t = 1.0f;
  uint32_t itr = 0U;
  _setupRK4Solver( &rk4config, &rk4input, &rk4output );
  
  printf( "# %s V %d.%d.%d\n", argv[0], VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH );
  printf( "# t s1     s2     s3     s4\n" );
  
  for ( itr = 0U; itr < 60; itr++ )
  {
    if ( RK4SOLVER_Solve( &rk4config, &rk4input, &rk4output ) == 1U )
    {
      printf( "%2.1f %5.4f %5.4f %5.4f %5.4f\n", t, rk4output.nextOutput[ 0 ], rk4output.nextOutput[ 1 ], rk4output.nextOutput[ 2 ], rk4output.nextOutput[ 3 ] );
      t += rk4input.h;
    }
    else
    {
      printf( "didn't work\n" );
      break;
    }
  }
  
  
  
  return 0;
}

void _setupRK4Solver( RK4SOLVER_CONFIGURATION * config, RK4SOLVER_INPUT * input, RK4SOLVER_OUTPUT * output )
{
  config->numStates = NUM_STATES;
  config->numInputs = NUM_INPUTS;
  config->numOutputs = NUM_OUTPUTS;
  config->A = (float*)A;
  config->B = (float*)B;
  config->C = (float*)C;
  config->D = (float*)D;
  
  input->h = 1.0;
  input->currentState = (float*)CurrentState;
  input->currentInput = (float*)CurrentInput;
  input->nextInput = (float*)CurrentInput;
  
  output->nextState = (float*)CurrentState;
  output->nextOutput = (float*)NextOutput;
  
}