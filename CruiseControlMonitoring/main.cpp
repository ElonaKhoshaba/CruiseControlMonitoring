#include "Monitor.h"
#include "Constants.h"
#include <iostream>
using namespace std;

int main()
{
	CruiseControllerMonitor monitor(DATA_PATH);
	// monitor.printAllData();

	// Overview
	monitor.printConstants();
	int numFaults = monitor.getNumFaults(); // Returns if you want to further manipulate numFaults
	monitor.printErrorBreakDown();

	// Details
	monitor.printTransientPeriods();
	monitor.printSteadyStatePeriods();
	monitor.printHillTimeImpacts();
	monitor.printElevationTimeIntervals();
	monitor.writeToControllerData();
}
