#include "hal/sim/RobotPhysics.h"
#include <cmath>

namespace {
// The right motor is mounted mirror-image, so positive drive spins its wheel BACKWARDS,
// exactly like on the real robot. (That is why config.h has kRightMotorInverted.)
constexpr double kMotorDirection[2] = { +1.0, -1.0 };

// Smooth version of "friction opposes motion" (avoids jitter around zero speed).
double frictionTorque(double friction, double speed) {
	return -friction * std::tanh(speed / 0.2);
}
}  // namespace

RobotPhysics::RobotPhysics(const RobotParams& params) : p_(params) {}

void RobotPhysics::setMotorDrive(int motor, double fraction, bool coast) {
	if (motor < 0 || motor > 1) return;
	if (fraction > 1) fraction = 1;
	if (fraction < -1) fraction = -1;
	drive_[motor] = fraction;
	coast_[motor] = coast;
}

void RobotPhysics::setBodyMode(BodyMode mode) {
	if (mode == BodyMode::Free && mode_ != BodyMode::Free) {
		wheelsOnGround_ = true;   // a free robot stands on its wheels
		xDot_ = 0;
	}
	if (mode == BodyMode::Held) {
		fallen_ = false;          // picking it up again
		heldAngle_ = theta_;
	}
	mode_ = mode;
}

void RobotPhysics::setHeldAngle(double thetaRad) { heldAngle_ = thetaRad; }
void RobotPhysics::setWheelsOnGround(bool onGround) { wheelsOnGround_ = onGround; }
void RobotPhysics::setTheta(double thetaRad) { theta_ = thetaRad; heldAngle_ = thetaRad; thetaDot_ = 0; }

// Torque one motor puts on its wheel, given how fast the wheel turns relative to the body.
// Simple DC motor line: full stall torque at 0 speed, zero torque at no-load speed.
double RobotPhysics::motorTorque(int motor, double relativeWheelSpeed) const {
	if (coast_[motor]) {
		return 0.0;
	}
	const double dir = kMotorDirection[motor];
	const double voltage = drive_[motor] * p_.supplyVoltage;
	const double shaftSpeed = dir * relativeWheelSpeed;
	const double shaftTorque = p_.motorStallTorque * (voltage / p_.motorRatedVoltage - shaftSpeed / p_.motorNoLoadSpeed);
	return dir * shaftTorque;
}

void RobotPhysics::step(double dt) {
	const double r = p_.wheelRadius;
	const double wheelInertiaEach = 0.5 * p_.wheelMass * r * r + p_.gearboxInertia;

	if (mode_ == BodyMode::Held) {
		// The hand smoothly follows the angle the scenario asks for (a stiff spring +
		// damper), so speeds and accelerations stay realistic instead of jumping.
		const double handStiffness = 60.0;   // rad/s, how quickly the hand responds
		thetaDDot_ = handStiffness * handStiffness * (heldAngle_ - theta_) - 2.0 * handStiffness * thetaDot_;
		thetaDot_ += thetaDDot_ * dt;
		theta_ += thetaDot_ * dt;
		xDot_ = 0;   // held in place
		xDDot_ = 0;

		for (int m = 0; m < 2; m++) {
			if (wheelsOnGround_) {
				wheelSpeed_[m] = 0;   // pressed against the floor, can't turn
				continue;
			}
			const double relative = wheelSpeed_[m] - thetaDot_;
			const double torque = motorTorque(m, relative) + frictionTorque(p_.gearboxFriction, relative);
			wheelSpeed_[m] += torque / wheelInertiaEach * dt;
		}
		return;
	}

	// ---------------- Free mode ----------------
	if (fallen_) {
		xDDot_ = -5.0 * xDot_;          // body scraping on the floor slows it down
		xDot_ += xDDot_ * dt;
		x_ += xDot_ * dt;
		thetaDot_ = 0;
		thetaDDot_ = 0;
		wheelSpeed_[0] = wheelSpeed_[1] = xDot_ / r;
		return;
	}

	const double M = p_.bodyMass;
	const double h = p_.comHeight;
	const double mw = 2.0 * p_.wheelMass;
	const double Iw = 2.0 * wheelInertiaEach;
	const double relative = xDot_ / r - thetaDot_;

	double torque = 0;   // total torque on the wheels (the body feels the opposite)
	for (int m = 0; m < 2; m++) {
		torque += motorTorque(m, relative) + frictionTorque(p_.gearboxFriction, relative);
	}

	// Equations of motion (Lagrange) for a pendulum on wheels:
	//   a11*xdd + a12*thdd = b1
	//   a12*xdd + a22*thdd = b2
	const double s = std::sin(theta_);
	const double c = std::cos(theta_);
	const double a11 = M + mw + Iw / (r * r);
	const double a12 = M * h * c;
	const double a22 = p_.bodyInertia + M * h * h;
	const double b1 = M * h * thetaDot_ * thetaDot_ * s + torque / r;
	const double b2 = M * p_.gravity * h * s - torque;
	const double det = a11 * a22 - a12 * a12;

	xDDot_ = (b1 * a22 - a12 * b2) / det;
	thetaDDot_ = (a11 * b2 - a12 * b1) / det;

	// Semi-implicit Euler: update speeds first, then positions.
	xDot_ += xDDot_ * dt;
	thetaDot_ += thetaDDot_ * dt;
	x_ += xDot_ * dt;
	theta_ += thetaDot_ * dt;
	wheelSpeed_[0] = wheelSpeed_[1] = xDot_ / r;

	if (std::fabs(theta_) >= p_.fallenAngle) {
		theta_ = theta_ > 0 ? p_.fallenAngle : -p_.fallenAngle;
		fallen_ = true;
	}
}
