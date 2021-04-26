#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
using namespace std;

///////////////
// Constants //
///////////////
static const string DATA_PATH = "C:/Users/elona/Desktop/ControllerMonitor/Result.txt";
static const float SAMPLING_RATE = 0.5;		// seconds; Rate at which SP changes - used to calculate avg accel
static const float STEP_INTERVAL = 0.1;		// seconds; Rate at which data is measured 
static const int NUM_DATA_SAMPLES = 6000;	// number of rows measured

// Thresholds
static const int RISE_TIME_THRESHOLD = 10;				// seconds; PV should rise to SP within this time
static const int SETTLING_TIME_ERROR_PERCENTAGE = .05;	// +-5%; This can be changed to +-2% for more precision.
static const int INFINITY_S = -999;

//////////////////////
// Monitoring Class //
//////////////////////
class CruiseControllerMonitor
{
	public:
		CruiseControllerMonitor(string filePath);
		~CruiseControllerMonitor();
		bool writeToControllerData();
		
		void printAllData();				// Prints preprocessed data
		
		void printTransientPeriods();
		void printSteadyStatePeriods();
		void printElevationTimeIntervals();	// Periods of flat, increasing, and decreasing elevation

	private:
		string m_filePath;					// Result.txt path
		string m_header;				    // Header of data
		int m_lines;						// Lines of data
		
		////////////////
		// Given data //
		////////////////
		vector<float> m_time;				// [s]
		vector<float> m_setpoint;			// [m/s]
		vector<float> m_measurement;		// [m/s]
		vector<float> m_longitudinalPos;	// [m]
		vector<float> m_elevation;			// [m]
		vector<float> m_controllerOutput;	// [N] 

		//////////////////
		// Derived Data //
		//////////////////
		vector<float> m_accel;														// index, index, value
		vector<vector<float>> m_transient;		// Transient time indices to accel index [a, b, accel] float b/c avg accel
		vector<vector<float>> m_steadyState;	// Steady state time indices to velocity index  [a, b, setpoint, riseTime, steadystateError]
																							// index, index, index, value
		
		vector<vector<int>> m_elevationChangeIndices;	// Indices of significant elevation periods
		vector<vector<float>> osciIntervalsForElevationChangesInSteadyStatePeriods;

		char* m_faultStatus;	        	// Array of fault statuses [+ or -]
		int m_faultCount;
		// Elevation changes -> time intervals 
		// Corresponding PV time intervals

		//////////////////////
		// Helper Functions //
		//////////////////////
		bool loadControllerData(string filePath);
		void calculateAccel();
		void calculatePeriods();
		void calculateElevationChangeTimeIntervals();
		void triggerFault(int start, int end);

		// Debugging
		void printAccel();

		//////////////////////////
		// Performance Analysis //
		//////////////////////////
		void calculateSettlingTimes();
		void calculateRiseTimes();
		void oscillationIntervalsForElevationChangesInSteadyStatePeriods();
		void maxUnderOverGivenTimeIntandSetpoint(float t1, float t2, float setpoint);
};

// Constructor
CruiseControllerMonitor::CruiseControllerMonitor(string filePath)
	: m_filePath(filePath)
{
	// Initialize member variables 
	loadControllerData(filePath);
	m_faultStatus = new char[m_lines];
	for (int i = 0; i < m_lines; i++)
	{
		m_faultStatus[i] = 'n';
	}
	calculateAccel();
	calculatePeriods();
	calculateElevationChangeTimeIntervals();						// not done
	oscillationIntervalsForElevationChangesInSteadyStatePeriods();	// not done
	calculateRiseTimes();
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

// Writes fault statuses to data file
bool CruiseControllerMonitor::writeToControllerData()
{
	// If file opening fails, do nothing
	ofstream saveFile(m_filePath);
	if (!saveFile)
		return false;

	string newHeader = m_header + ", FaultStatus [y/n]\n";
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
				m_steadyState.push_back({float(i - count2 + 1), float(i), float(i)});
			count2 = 0;
		}
		else // accel == 0
		{
			// Transient
			if (sum != 0 && count > 0)
			{
				avgAccel = sum / count;
				m_transient.push_back({float(i - count), float(i + 1), avgAccel});
			}
			sum = 0;
			count = 0;

			// Steady state
			if (i == m_accel.size() - 1)
				m_steadyState.push_back({float(i - count2 + 1), float(NUM_DATA_SAMPLES - 1), float(i)});	// time, time, setpoint
			count2++;
		}
	}
}

// Assumes that initial condition is flat
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
			m_elevationChangeIndices.push_back({t1, t2});
		}
		if (curr_elevation_change == 0 && next_elevation_change != 0)
		{
			flat_t2 = i + 1;
			
			//if (flat_t1 != 0)	// Excludes first interval where the elevation is initally unchanging
			//{
			m_elevationChangeIndices.push_back({flat_t1, flat_t2});
				// cout << "Flat: " << flat_t1 << " " << flat_t2 << endl;
			//}
			t1 = i + 1;
		}
	}
	// Rest of flat section
	flat_t2 = NUM_DATA_SAMPLES - 1;
	// cout << "Flat: " << flat_t1 << " " << flat_t2 << endl;
	m_elevationChangeIndices.push_back({flat_t1, flat_t2});
}

void CruiseControllerMonitor::triggerFault(int start, int end)
{
	for (int k = start; k < end; k++)
	{
		m_faultStatus[k] = 'y';
		m_faultCount++;
	}
}


////////////////////////////////////
// Performance Analysis Functions //
////////////////////////////////////

// Settling Time: the time required for the PV's damped oscillations to settle within a certain
// percentage of the steady-state value (commonly  +-2% or +-5% of the steady-state value)
void CruiseControllerMonitor::calculateSettlingTimes()
{

}
// Percent Overshoot: the amount that the process variable overshoots the final value,
// expressed as a percentage of the final value. 
// Settling time: the time required for the process  variable to settle to within a certain percentage (commonly 2% or 5%) 
// of the final value. 
// Steady-State Error: the final difference between the process variable and setpoint

// Calculates the relative rise time for transient periods
// Rise Time: the amount of time the system takes to go from 10% to 90% of the target steady-state value.*
// * Percentages from relative setpoint changes (currSetpoint - lastSetpoint)
// vector<vector<float>> m_steadyState -> Steady state interval to velocity [a, b, vel]
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
	/*	Absolute percentage
		float vf_10percent = v_final * 0.1;
		float vf_90percent = v_final * 0.9;		*/
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
		if (m_measurement[j] < vf_90percent)
		{
			// cout << "Rise time is infinite -> measurement does not reach within 90% of relative setpoint change.\nPV " << m_measurement[j] << "m/s vs Upper limit" << vf_90percent << "m/s" << endl;
			// cout << "Time taken: infinity s" << endl << endl;

// ----->   // TRIGGERING RISE TIME FAULTS HERE
			m_transient[i].push_back(INFINITY_S);
			triggerFault(j, NUM_DATA_SAMPLES);
			// IMCOMPLETE - take average to find steadystate
			float steadyStateMeasurement = m_measurement[m_steadyState[i][1]];
			// cout << "Steadystate measurement" << v_final - steadyStateMeasurement << endl;
			m_steadyState[i].push_back(v_final - steadyStateMeasurement);	// steady state error SP - PV

		}
		else
		{
			// cout << "Time taken: " << STEP_INTERVAL * count << "s" << endl << endl;
			float riseTime = STEP_INTERVAL * count;
			// If calculated rise time is above set threshold, trigger fault
			if (riseTime > RISE_TIME_THRESHOLD)
				triggerFault(j, NUM_DATA_SAMPLES);
		
			m_transient[i].push_back(riseTime);
			m_steadyState[i].push_back(0);	// No steady state error
		}
	}
}

void CruiseControllerMonitor::oscillationIntervalsForElevationChangesInSteadyStatePeriods()
{
	for (int i = 0; i < m_steadyState.size(); i++)
	{
		for (int j = 0; j < m_elevationChangeIndices.size(); j++)
		{
			float t1 = m_time[m_elevationChangeIndices[j][0]];
			float t2 = m_time[m_elevationChangeIndices[j][1]];
			float min = m_steadyState[i][0];
			float max = m_steadyState[i][1];
			// If elevation change is within steady state period
			if (t1 >= min)
			{
				// If elevation interval runs past current steady state period, set it to the upper limit 
				// of the period
				if (t2 > max)
					t2 = max;
				if (i == 0)
					cout << "22: " << t1 << " " << t2 << endl;
				// Find under or overshooting value for given time interval and setpoint
				maxUnderOverGivenTimeIntandSetpoint(t1, t2, m_steadyState[i][2]);
			}
		}
	}
	cout << endl;
}

// If the velocity undershoots or overshoots past an experimentally determined error margin, trigger a fault
// at said indice 
void CruiseControllerMonitor::maxUnderOverGivenTimeIntandSetpoint(float t1, float t2, float setpoint)
{
	
}


////////////////////////
// Printing functions //
////////////////////////
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
	cout << "Time interval [s,s] : Setpoint [m/s] : Rise time [m/s]" << endl;
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
			<< m_setpoint[m_steadyState[i][2]] << " m/s : " << m_steadyState[i][3] << "m/s\n";

	}
	cout << endl;
}

void CruiseControllerMonitor::printElevationTimeIntervals()
{
	cout << "Signifcant elevation time intervals: " << endl;
	for (int i = 0; i < m_elevationChangeIndices.size(); i++)
	{
		cout << "[" << m_time[m_elevationChangeIndices[i][0]] << "s, " << m_time[m_elevationChangeIndices[i][1]] << "s]" << endl;
	}
	cout << endl;
}


int main()
{
	CruiseControllerMonitor monitor(DATA_PATH);
	// monitor.printAllData();
	 monitor.printTransientPeriods();
	 monitor.printSteadyStatePeriods();
	 monitor.printElevationTimeIntervals();
	 // monitor.writeToControllerData();
}