#pragma once
#include "Constants.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

//////////////////////
// Monitoring Class //
//////////////////////

class CruiseControllerMonitor
{
	public:
		 CruiseControllerMonitor(std::string filePath);
		~CruiseControllerMonitor();

		// Writes postprocessed data to result file
		bool writeToControllerData();
	
		/////////////////////////////
		// Printing/User Functions //
		/////////////////////////////

		// Prints preprocessed data
		void printAllData();	
		void printTransientPeriods();
		void printSteadyStatePeriods();
		// Periods of flat, increasing, and decreasing elevation
		void printElevationTimeIntervals();
		void printHillTimeImpacts();
		void printConstants();
		int getNumFaults();
		void printErrorBreakDown();

	private:

		std::string m_filePath;				// Result.txt path
		std::string m_header;				// Header of data
		int m_lines;						// Lines of data

		////////////////
		// Given data //
		////////////////

		std::vector<float> m_time;				// [s]
		std::vector<float> m_setpoint;			// [m/s]
		std::vector<float> m_measurement;		// [m/s]
		std::vector<float> m_longitudinalPos;	// [m]
		std::vector<float> m_elevation;			// [m]
		std::vector<float> m_controllerOutput;	// [N] 

		//////////////////
		// Derived Data //
		//////////////////

		// Calculated* accleration values
		// *Sampling rate used, NOT the step interval
		std::vector<float> m_accel;															

		// Transient time indices to accel index [a, b, accel, riseTime] float b/c avg accel
		// Index Key: [dataIndex1, dataIndex2, accelSP, riseTime] 
		std::vector<std::vector<float>> m_transient;	
		
		// Steady state time indices to velocity index 
		// Index Key: [dataIndex1, dataIndex2, velocitySP, steadyStateError]
		std::vector<std::vector<float>> m_steadyState;

		// Indices of oscillations caused by hills
		// Index Key: [dataIndex1, dataIndex2, settlingTime from dataIndex1]
		std::vector<std::vector<float>> m_hillIndices;																
																						
		// Indices of significant elevation periods
		// Index Key: [dataIndex1, dataIndex2]
		std::vector<std::vector<float>> m_elevationChangeIndices;

		// SP - PV
		std::vector<float> m_rawError;

		// Array of fault statuses [0 or 1]
		int* m_faultStatus;	  

		// Tracks number of faults
		int m_faultCount;	

		// Error breakdown
		int m_riseTimeFaults;
		int m_settlingTimeFaults;
		int m_rawErrorFaults;
		float m_rawErrorFraction;
		float m_riseTimeFraction;
		float m_settlingTimeFraction;
	
		//////////////////////////
		// Performance Analysis //
		//////////////////////////
		void calculateSettlingTimesOfHills();
		void calculateRiseTimes();
		void calculateRawError();

		//////////////////////
		// Helper Functions //
		//////////////////////
		bool loadControllerData(std::string filePath);
		void calculateAccel();
		void calculatePeriods();
		void calculateElevationChangeTimeIntervals();
		void calcHillOsccilationIntervals();
		void triggerFault(int start, int end, int key);
		void calcErrorBreakDown();

		// void maxUnderOverGivenTimeIndexandSetpoint(int a, int b, float setpoint);	// Not implemented

		// Debugging
		void printAccel();
};

