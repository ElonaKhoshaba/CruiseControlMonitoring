#include "Monitor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
using namespace std;

//////////////////////////////////
// Monitor Class Implementation //
//////////////////////////////////

// Constructor
CruiseControllerMonitor::CruiseControllerMonitor(string filePath)
	: m_filePath(filePath)
{
	// Initialize member variables 
	loadControllerData(filePath);
	m_faultStatus = new int[m_lines];
	for (int i = 0; i < m_lines; i++)
	{
		m_faultStatus[i] = 0;
	}
	calculateAccel();
	calculatePeriods();
	calculateElevationChangeTimeIntervals();
	calcHillOsccilationIntervals();
	calculateRiseTimes();
	calculateSettlingTimesOfHills();
	calculateRawError();
	calcErrorBreakDown();
}

// Destructor
CruiseControllerMonitor::~CruiseControllerMonitor()
{
	delete[] m_faultStatus;
}

// Loads result data to member variables
bool CruiseControllerMonitor::loadControllerData(string file)
{
	// If opening the file fails do nothing
	ifstream dataFile(file);
	if (!dataFile)
		return false;

	// Save first line as header
	string line;
	getline(dataFile, line);
	m_header = line;

	// Load the file into data
	string time, setpoint, measurement, pos, elevation, output;
	while (getline(dataFile, line))		// While not at end of file
	{
		stringstream currLine(line);
		getline(currLine, time, ',');
		m_time.push_back(stof(time));

		getline(currLine, setpoint, ',');
		m_setpoint.push_back(stof(setpoint));

		getline(currLine, measurement, ',');
		m_measurement.push_back(stof(measurement));

		getline(currLine, pos, ',');
		m_longitudinalPos.push_back(stof(pos));

		getline(currLine, elevation, ',');
		m_elevation.push_back(stof(elevation));

		getline(currLine, output, '\n');				// New line after controller output in Result.txt
		m_controllerOutput.push_back(stof(output));
	}

	m_lines = m_time.size();
	return true;
}

// Writes fault statuses to data file
bool CruiseControllerMonitor::writeToControllerData()
{
	// If file opening fails, do nothing
	ofstream saveFile(m_filePath);
	if (!saveFile)
		return false;

	string newHeader = m_header + ", FaultStatus [0/1]\n";
	saveFile << newHeader;
	for (int i = 0; i < m_lines; i++)
	{
		saveFile << m_time[i] << ", " << m_setpoint[i] << ", " << m_measurement[i] << ", " << m_longitudinalPos[i] << ", " << m_elevation[i]
			<< ", " << m_controllerOutput[i] << ", " << m_faultStatus[i] << endl;
	}
	return true;
}

// Calculates the acceleration for each 0.5s period (since the setpoint changes every 0.5 seconds)
void CruiseControllerMonitor::calculateAccel()
{
	for (int i = 0; i < m_lines - 5; i++)
		m_accel.push_back((m_setpoint[i + 5] - m_setpoint[i]) / SAMPLING_RATE);
}

// Calculates and stores the time interval of transient and steady-state period throughout controller time history
void CruiseControllerMonitor::calculatePeriods()
{
	// For transient
	float sum = 0;
	float avgAccel = 0;
	int count = 0;
	// For steady-state
	int count2 = 0;
	for (int i = 0; i < m_accel.size(); i++)
	{
		// Transient periods
		if (m_accel[i] != 0)
		{
			count++;
			sum += m_accel[i];
			// Steadystate
			if (count2 > 0)
				m_steadyState.push_back({ float(i - count2 + 1), float(i), float(i) });
			count2 = 0;
		}
		else // accel == 0
		{
			// Transient
			if (sum != 0 && count > 0)
			{
				avgAccel = sum / count;
				m_transient.push_back({ float(i - count), float(i + 1), avgAccel });
			}
			sum = 0;
			count = 0;

			// Steady state
			if (i == m_accel.size() - 1)
				m_steadyState.push_back({ float(i - count2 + 1), float(NUM_DATA_SAMPLES - 1), float(i) });	// time, time, setpoint
			count2++;
		}
	}
}

// Stores intervals of elevation periods (const, rising, decreasing) into vector
void CruiseControllerMonitor::calculateElevationChangeTimeIntervals()
{
	int t1 = 0;
	int t2 = 0;
	int flat_t1 = 0;
	int flat_t2 = 0;
	for (int i = 0; i < m_lines - 2; i++)
	{
		float curr_elevation_change = (m_elevation[i + 1] - m_elevation[i]) / STEP_INTERVAL;
		float next_elevation_change = (m_elevation[i + 2] - m_elevation[i + 1]) / STEP_INTERVAL;
		if (curr_elevation_change != 0 && next_elevation_change == 0)
		{
			t2 = i + 1;
			flat_t1 = i + 1;
			// cout << "Change: " << t1 << " " << t2 << endl;
			m_elevationChangeIndices.push_back({ float(t1), float(t2) });
		}
		if (curr_elevation_change == 0 && next_elevation_change != 0)
		{
			flat_t2 = i + 1;

			//if (flat_t1 != 0)	// Excludes first interval where the elevation is initally unchanging
			//{
			m_elevationChangeIndices.push_back({ float(flat_t1), float(flat_t2) });
			// cout << "Flat: " << flat_t1 << " " << flat_t2 << endl;
		//}
			t1 = i + 1;
		}
	}
	// Rest of flat section
	flat_t2 = NUM_DATA_SAMPLES - 1;
	// cout << "Flat: " << flat_t1 << " " << flat_t2 << endl;
	m_elevationChangeIndices.push_back({ float(flat_t1), float(flat_t2) });
}

// Updates faultStatus vector
void CruiseControllerMonitor::triggerFault(int start, int end, int key)
{
	if (start == end)
	{
		m_faultCount++;
		switch (key)
		{
		case RISE_TIME:
			m_riseTimeFaults++;
			break;
		case SETTLING_TIME:
			m_settlingTimeFaults++;
			break;
		case RAW_ERROR:
			m_rawErrorFaults++;
			break;
		}
		return;
	}

	for (int k = start; k < end; k++)
	{
		if (m_faultStatus[k] == 1)
			continue;
		m_faultStatus[k] = 1;
		m_faultCount++;
		switch (key)
		{
		case RISE_TIME:
			m_riseTimeFaults++;
			break;
		case SETTLING_TIME:
			m_settlingTimeFaults++;
			break;
		case RAW_ERROR:
			m_rawErrorFaults++;
			break;
		}
	}
}


////////////////////////////////////
// Performance Analysis Functions //
////////////////////////////////////

// Calculates settling time of the oscillating measured velocity caused by changes in elevation 
// Settling Time: the time required for the PV's damped oscillations to settle within a certain
// percentage of the steady-state value (commonly  +-2% or +-5% of the steady-state value)
void CruiseControllerMonitor::calculateSettlingTimesOfHills()
{
	// Error bar limits
	// Assuming setpoint is constant during hills
	float setpoint = m_setpoint[m_hillIndices[0][0]];
	float lowerBound = setpoint - (setpoint * SETTLING_TIME_ERROR_PERCENTAGE);
	float upperBound = setpoint + (setpoint * SETTLING_TIME_ERROR_PERCENTAGE);
	bool consec = true;
	// For each hill interval, find the settling times
	for (int i = 0; i < m_hillIndices.size(); i++)
	{
		int j;
		// cout << "From: " << m_hillIndices[i][0] << " to " << m_hillIndices[i][1] + 1 << endl;
		for (j = m_hillIndices[i][0]; j < m_hillIndices[i][1] + 1; j++)
		{
			if (m_measurement[j] >= lowerBound && m_measurement[j] <= upperBound)
			{
				int count = 0;
				for (int k = j; k < j + SETTLING_TIME_CONSECUTIVE; k++)
				{
					if (!(m_measurement[k] >= lowerBound && m_measurement[k] <= upperBound))
						break;
					else
						count++;
				}
				if (count == SETTLING_TIME_CONSECUTIVE)
					break;
			}
		}
		if (j == m_hillIndices[i][1] + 1)
		{
			triggerFault(j, m_hillIndices[i][1] + 1, SETTLING_TIME);
			m_hillIndices[i].push_back(INFINITY_S);
			// cout << "No Settling Time" << endl;
		}

		else
		{
			// cout << "First Time Match: " << m_time[j] << "s" << endl;
			// cout << "Settling Time: " << m_time[j] - m_time[m_hillIndices[i][0]] << "s" << endl;
			float settlingTime = m_time[j] - m_time[m_hillIndices[i][0]];
			if (settlingTime > SETTLING_TIME_THRESHOLD)
			{
				triggerFault(j, m_hillIndices[i][1] + 1, SETTLING_TIME);
				// cout << "Settling Time Treshold Passed\n" << endl;
			}
			m_hillIndices[i].push_back(settlingTime);
		}
		// cout << endl;
	}
}


// Calculates the relative rise time for transient periods and average steady state error if it exists
// Rise Time: the amount of time the system takes to go from 10% to 90% of the target steady-state value.*
//		* Percentages from relative setpoint changes (currSetpoint - lastSetpoint)
// Steady-State Error: the final difference between the process variable and setpoint
void CruiseControllerMonitor::calculateRiseTimes()
{
	for (int i = 0; i < m_steadyState.size(); i++)
	{
		// Calculate reltive setpoint change and 10-90% range
		float v_final = m_setpoint[m_steadyState[i][2]];
		float v_initial;
		if (i == 0)
			v_initial = m_setpoint[0];
		else
			v_initial = m_setpoint[m_steadyState[i - 1][2]];
		float vf_10percent = ((v_final - v_initial) * 0.1) + v_initial;
		float vf_90percent = ((v_final - v_initial) * 0.9) + v_initial;
		// cout << "Setpoint: " << v_final << "m/s" << endl;
		// cout << "10% of SP: " << vf_10percent << "m/s\n90% of SP: " << vf_90percent << "m/s" << endl;
		// Amount of time the PV takes to get from vf_10percent to vf_90percent during corresponding transient intervals
		// cout << m_transient[i][0] << " " << m_transient[i][1] << endl;
		int count = 0;
		int j;
		for (j = m_transient[i][0]; j < m_transient[i][1] + 1; j++)	// Add extra 1 to get entire time interval
		{
			// If PV is between 10 to 90 percent of final value
			if (m_measurement[j] >= vf_10percent && m_measurement[j] <= vf_90percent)
			{
				// cout << m_measurement[j-1] << "m/s " << m_time[j] << "s" << endl;
				count++;
			}
			if (m_measurement[j] >= vf_90percent)
			{
				count++;	// Round up
				break;
			}
		}

		// Measurement does not reach within 90% of relative setpoint change --> rise time infinite
		if (m_measurement[j] < vf_90percent)
		{
			// cout << "Rise time is infinite -> measurement does not reach within 90% of relative setpoint change.\nPV " << m_measurement[j] << "m/s vs Upper limit" << vf_90percent << "m/s" << endl;
			// cout << "Time taken: infinity s" << endl << endl;

// ----->   // TRIGGERING RISE TIME FAULTS HERE
			m_transient[i].push_back(INFINITY_S);
			triggerFault(j - 1, m_steadyState[i][1] + 1, RISE_TIME);

			// Finding average steady state error
			float sum = 0;
			for (int k = m_steadyState[i][0]; k < m_steadyState[i][1] + 1; k++)
				sum += m_measurement[k];
			float average = sum / ((m_steadyState[i][1] + 1) - m_steadyState[i][0]);
			m_steadyState[i].push_back(v_final - average);	// steady state error SP - PV
		}
		else
		{
			// cout << "Time taken: " << STEP_INTERVAL * count << "s" << endl << endl;
			float riseTime = STEP_INTERVAL * count;
			// If calculated rise time is above set threshold, trigger fault
			if (riseTime > RISE_TIME_THRESHOLD)
			{
				triggerFault(m_transient[i][0], m_transient[i][1] + 1, RISE_TIME);
			}

			m_transient[i].push_back(riseTime);
			m_steadyState[i].push_back(0);	// No steady state error
		}
	}
}

// Calculates periods of measured veloctity oscillation caused by hills
void CruiseControllerMonitor::calcHillOsccilationIntervals()
{
	for (int i = 0; i < m_steadyState.size(); i++)
	{
		for (int j = 0; j < m_elevationChangeIndices.size(); j++)
		{
			float t1 = m_time[m_elevationChangeIndices[j][0]];
			float t2 = m_time[m_elevationChangeIndices[j][1]];
			float min = m_time[m_steadyState[i][0]];
			float max = m_time[m_steadyState[i][1]];
			// If elevation change is within steady state period
			if (t1 >= min)
			{
				// If elevation interval runs past current steady state period, set it to the upper limit 
				// of the period
				if (t2 > max)
					m_hillIndices.push_back({ m_elevationChangeIndices[j][0], m_steadyState[i][1] });
				else
					m_hillIndices.push_back({ m_elevationChangeIndices[j][0], m_elevationChangeIndices[j][1] });

// --------->   // UNFINISHED: Find under or overshooting value for given time interval and setpoint
				// maxUnderOverGivenTimeIndexandSetpoint(m_elevationChangeIndices[j][0], m_elevationChangeIndices[j][0], m_steadyState[i][2]);
			}
		}
	}
}

void CruiseControllerMonitor::calculateRawError()
{
	for(int i = 0; i < NUM_DATA_SAMPLES; i++)
	{
		float difference = m_setpoint[i] - m_measurement[i];
		m_rawError.push_back(difference);
		// cout << fabs(difference / m_setpoint[i]) << " " << RAW_ERROR_THRESHOLD << endl;
		if (fabs(difference / m_setpoint[i]) > RAW_ERROR_THRESHOLD)
		{
			if (m_faultStatus[i] == 1)	// Don't count a fault twice
				continue;
			m_faultStatus[i] = 1;
			m_faultCount++;
			m_rawErrorFaults++;
		}
	}
}

void CruiseControllerMonitor::calcErrorBreakDown()
{
	m_rawErrorFraction = float(m_rawErrorFaults) / NUM_DATA_SAMPLES;
	m_settlingTimeFraction = float(m_settlingTimeFaults) / NUM_DATA_SAMPLES;
	m_riseTimeFraction = float(m_riseTimeFaults) / NUM_DATA_SAMPLES;
}


////////////////////////////
// UNFINISHED DEVELOPMENT //
////////////////////////////

// If the velocity undershoots or overshoots past an experimentally determined error margin, trigger a fault
// at said indice 
/* void CruiseControllerMonitor::maxUnderOverGivenTimeIndexandSetpoint(int a, int b, float setpoint)
{

} */

// Percent Overshoot: the amount that the process variable overshoots the final value,
// expressed as a percentage of the final value. 

// Raw absolute error?

/////////////////////////////
// Printing/User functions //
/////////////////////////////
void CruiseControllerMonitor::printAccel()
{
	for (int i = 0; i < m_accel.size(); i++)
	{
		if (m_accel[i] != 0)
			cout << m_accel[i] << ", " << i << endl;
	}
}

void CruiseControllerMonitor::printTransientPeriods()
{
	cout << "Transient Periods: " << endl;
	cout << "Time interval [s,s] : Setpoint [m/s^2] : Rise time [s]" << endl;
	for (int i = 0; i < m_transient.size(); i++)
	{
		cout << "[" << m_time[m_transient[i][0]] << "s, " << m_time[m_transient[i][1]] << "s] : "
			<< m_transient[i][2] << " m/s^2 : ";
		if (m_transient[i][3] == INFINITY_S)
			cout << "INFINITY";
		else
			cout << m_transient[i][3];
		cout << endl;
	}
	cout << endl;
}

void CruiseControllerMonitor::printSteadyStatePeriods()
{
	cout << "Steady-state Periods: " << endl;
	cout << "Time interval [s,s] : Setpoint [m/s] : Steady-state error [m/s]" << endl;
	for (int i = 0; i < m_steadyState.size(); i++)
	{
		cout << "[" << m_time[m_steadyState[i][0]] << "s, " << m_time[m_steadyState[i][1]] << "s] : "
			<< m_setpoint[m_steadyState[i][2]] << " m/s : " << m_steadyState[i][3] << " m/s\n";

	}
	cout << endl;
}

void CruiseControllerMonitor::printElevationTimeIntervals()
{
	cout << "General Elevation Time Intervals: " << endl;
	for (int i = 0; i < m_elevationChangeIndices.size(); i++)
	{
		cout << "[" << m_time[m_elevationChangeIndices[i][0]] << "s, " << m_time[m_elevationChangeIndices[i][1]] << "s]" << endl;
	}
	cout << endl;
}

void CruiseControllerMonitor::printHillTimeImpacts()
{
	cout << "Settling Times of Elevation-Induced Velocity Oscillations: " << endl;
	cout << "Elevation Time Interval [s,s] : Settling time [s]" << endl;
	for (int i = 0; i < m_hillIndices.size(); i++)
	{
		cout << "[" << m_time[m_hillIndices[i][0]] << "s, " << m_time[m_hillIndices[i][1]] << "s] : " << m_hillIndices[i][2] << "s" << endl;
	}
	cout << endl;
}

// Prints pre-processed data
void CruiseControllerMonitor::printAllData()
{
	cout << m_header;
	for (int i = 0; i < m_lines; i++)
	{
		cout << m_time[i] << ", " << m_setpoint[i] << ", " << m_measurement[i] << ", " << m_longitudinalPos[i] << ", " << m_elevation[i]
			<< ", " << m_controllerOutput[i] << ", " << endl;
	}
}

void CruiseControllerMonitor::printConstants()
{
	cout << "Constants:" << endl;
	cout << "Data Samples: " << NUM_DATA_SAMPLES << " samples" << endl;
	cout << "Rise Time: " << RISE_TIME_THRESHOLD << "s" << endl;
	cout << "Settling Time: " << SETTLING_TIME_THRESHOLD << "s" << endl;
	cout << "Settling Time Consecutive Requirement: " << SETTLING_TIME_CONSECUTIVE << " measurements" << endl;
	cout << "Settling Time Error-band Percentage: " << SETTLING_TIME_ERROR_PERCENTAGE * 100 << "% " << endl;
	cout << endl;
}

int CruiseControllerMonitor::getNumFaults()
{
	cout << "Results:" << endl;
	cout << "Total faults: " << m_faultCount << endl; // " for " << NUM_DATA_SAMPLES << " data samples" << endl;
	cout << "Percentage of faults: " << (float(m_faultCount) / NUM_DATA_SAMPLES) * 100 << "%" << endl << endl; // "% [(count / samples) x 100]" << endl;
	return m_faultCount;
}

void CruiseControllerMonitor::printErrorBreakDown()
{
	cout << "Error Breakdown: " << endl;
	cout << "Percent error due to raw error: " << m_rawErrorFraction * 100 << "%" << endl;
	cout << "Percent error due to settling time: " << m_settlingTimeFraction * 100 << "%" << endl;
	cout << "Percent error due to rise time: " << m_riseTimeFraction * 100 << "%" << endl;
	cout << endl;
}