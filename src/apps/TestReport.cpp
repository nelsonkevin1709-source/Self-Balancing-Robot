#include "apps/TestReport.h"
#include <stdarg.h>
#include <stdio.h>

TestReport::TestReport(HardwareInterface& hw, const char* title) : hw_(hw), title_(title) {
	hw_.printf("\n========== %s TEST ==========\n", title_);
}

bool TestReport::check(const char* label, bool passed, const char* detailFormat, ...) {
	char detail[160] = "";
	if (detailFormat != nullptr) {
		va_list args;
		va_start(args, detailFormat);
		vsnprintf(detail, sizeof(detail), detailFormat, args);
		va_end(args);
	}
	hw_.printf("%s  %s%s%s\n", passed ? "PASS" : "FAIL", label, detail[0] ? "  -> " : "", detail);
	if (passed) passed_++;
	else failed_++;
	return passed;
}

int TestReport::askYesNo(const char* label, const char* question, const char* expectation, uint32_t timeoutMs) {
	// Throw away anything typed earlier, then give the wheels/sensor a second to settle.
	while (hw_.readChar() >= 0) {}
	hw_.delayMs(1000);

	hw_.printf("  ?   %s  (type y or n)\n", question);
	int answer = -1;
	const uint32_t start = hw_.millis();
	while (answer < 0 && hw_.millis() - start < timeoutMs) {
		const int c = hw_.readChar();
		if (c == 'y' || c == 'Y') answer = 1;
		else if (c == 'n' || c == 'N') answer = 0;
		else {
			const int seen = hw_.observe(expectation);   // only the simulator answers this
			if (seen >= 0) {
				hw_.printf("      [simulator looked: %s]\n", seen ? "yes" : "no");
				answer = seen;
			} else {
				hw_.delayMs(10);
			}
		}
	}

	if (answer < 0) {
		hw_.printf("SKIP  %s  -> no answer\n", label);
		skipped_++;
	} else {
		check(label, answer == 1);
	}
	return answer;
}

void TestReport::summary() {
	hw_.printf("---------- %s: %d passed, %d failed, %d skipped ----------\n\n", title_, passed_, failed_, skipped_);
}
