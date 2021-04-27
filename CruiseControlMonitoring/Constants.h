#pragma once
#include <string>

///////////////////////
// Monitor Constants //
///////////////////////

// Data Constants
static const std::string DATA_PATH = "C:/Users/elona/Desktop/ControllerMonitor/Result.txt";
static const int NUM_DATA_SAMPLES = 6000;	// Number of rows measured

// Controller Constants
static const float SAMPLING_RATE = 0.5;		// seconds; Rate at which SP changes - used to calculate avg accel
static const float STEP_INTERVAL = 0.1;		// seconds; Rate at which data is measured 

// Rise Time Constants (Transient)
static const float RISE_TIME_THRESHOLD = 20;				// seconds; PV should rise to SP within this time
static const int INFINITY_S = -999;							// Infinite (invalid) rise time

// Settling Time Constants (Steady-state)
static const float SETTLING_TIME_ERROR_PERCENTAGE = .05;	// +-5%; This can be changed to +-2% for more precision (or some other number).
static const int SETTLING_TIME_CONSECUTIVE = 50;			// Want XX consecutive measurements to be within error band (XX * step interval = 
static const float SETTLING_TIME_THRESHOLD = 15;		    // XX second settling time upper limit

// Raw Error
static const float RAW_ERROR_THRESHOLD = .1; // 0.07;		// XX% max deviation from setpoint 
															// Good range seems to be 5-15%

// Enumerated Names
static const int RISE_TIME = 0;
static const int SETTLING_TIME = 1;
static const int RAW_ERROR = 2;