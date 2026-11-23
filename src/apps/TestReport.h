/*****************
TestReport.h
Self-Balancing Robot

PASS / FAIL bookkeeping for the component tests, like the check() helper in
MTRN3500's GalilMain.cpp.

	TestReport report(hw, "MPU6050");
	report.check("chip found", imu.begin(), "WHO_AM_I = 0x%02X", id);
	report.askYesNo("left wheel forward", "Is the LEFT wheel spinning FORWARD?", "L+");
	report.summary();
*****************/

#pragma once
#include "hal/HardwareInterface.h"

class TestReport {
public:
	TestReport(HardwareInterface& hw, const char* title);

	// Records and prints one result. detailFormat is optional printf-style extra info.
	bool check(const char* label, bool passed, const char* detailFormat = nullptr, ...);

	// For things only a human can see. Prints the question and waits for 'y' or 'n'
	// typed in the serial monitor. In the simulator, 'expectation' (e.g. "L+") lets the
	// simulator answer by looking at its physics. No answer before timeout = SKIP.
	// Returns 1 yes, 0 no, -1 skipped.
	int askYesNo(const char* label, const char* question, const char* expectation, uint32_t timeoutMs = 20000);

	void summary();            // "4 passed, 0 failed, 1 skipped"
	int failures() const { return failed_; }

private:
	HardwareInterface& hw_;
	const char* title_;
	int passed_ = 0;
	int failed_ = 0;
	int skipped_ = 0;
};
