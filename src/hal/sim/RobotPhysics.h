/*****************
RobotPhysics.h
Self-Balancing Robot (simulator only)

A 2D model of the robot: a body (inverted pendulum) sitting on an axle with two
wheels, driven by two yellow TT gear motors. Side view only, so turning is not
modelled (both wheels share one ground position).

Sign conventions (used everywhere in the project):
    x      wheel position along the floor, + = forward            [m]
    theta  body lean angle, 0 = upright, + = leaning FORWARD       [rad]
    motor torque + = pushes the wheel to roll the robot forward

The numbers in RobotParams are estimates for a small TT-motor robot. They do
not need to be exact: the goal is a model that falls, drives and balances
roughly like the real thing.
*****************/

#pragma once

struct RobotParams {
	double bodyMass = 0.45;          // kg  chassis + ESP32 + battery (not wheels)
	double comHeight = 0.07;         // m   axle -> body centre of mass
	double bodyInertia = 6.0e-4;     // kg m^2 about the centre of mass
	double wheelRadius = 0.033;      // m   yellow TT wheel ~66 mm diameter
	double wheelMass = 0.03;         // kg  each
	double gearboxInertia = 2.0e-4;  // kg m^2 per wheel (motor rotor seen through the 1:48 gearbox)
	double motorStallTorque = 0.08;  // N m per motor at rated voltage
	double motorNoLoadSpeed = 21.0;  // rad/s at rated voltage (~200 rpm)
	double motorRatedVoltage = 6.0;  // V
	double gearboxFriction = 0.012;  // N m per motor (why small PWM values don't move the wheel)
	double supplyVoltage = 6.0;      // V  battery on the DRV8833 VM pin (4xAA)
	double imuHeight = 0.10;         // m  axle -> MPU6050
	double fallenAngle = 1.3;        // rad (~75 deg) body hits the floor
	double gravity = 9.81;
};

enum class BodyMode {
	Held,   // a "hand" holds the body at an angle we choose (scenario sets it)
	Free    // real physics: the robot can drive, balance and fall over
};

class RobotPhysics {
public:
	explicit RobotPhysics(const RobotParams& params = RobotParams());

	// One small time step. dt in seconds (the simulator uses 0.25 ms).
	void step(double dt);

	// --- inputs ---
	// Average H-bridge output for each motor as a fraction of supply voltage (-1..1).
	// coast = true means both driver inputs are LOW (motor disconnected, spins freely).
	void setMotorDrive(int motor, double fraction, bool coast);   // motor 0 = left, 1 = right

	// --- scenario controls ---
	void setBodyMode(BodyMode mode);
	void setHeldAngle(double thetaRad);         // used while Held
	void setWheelsOnGround(bool onGround);      // false = robot lifted, wheels spin in the air
	void setTheta(double thetaRad);             // teleport (e.g. starting lean when released)

	// --- state ---
	double x() const { return x_; }
	double xDot() const { return xDot_; }
	double xDDot() const { return xDDot_; }
	double theta() const { return theta_; }
	double thetaDot() const { return thetaDot_; }
	double thetaDDot() const { return thetaDDot_; }
	double wheelSpeed(int motor) const { return wheelSpeed_[motor]; }   // rad/s, + = forward
	double motorDrive(int motor) const { return coast_[motor] ? 0.0 : drive_[motor]; }   // -1..1 as wired
	bool hasFallen() const { return fallen_; }
	BodyMode bodyMode() const { return mode_; }
	const RobotParams& params() const { return p_; }

private:
	double motorTorque(int motor, double relativeSpeed) const;

	RobotParams p_;
	BodyMode mode_ = BodyMode::Held;
	bool wheelsOnGround_ = true;
	bool fallen_ = false;

	double x_ = 0, xDot_ = 0, xDDot_ = 0;
	double theta_ = 0, thetaDot_ = 0, thetaDDot_ = 0;
	double heldAngle_ = 0;
	double wheelSpeed_[2] = { 0, 0 };   // used when wheels are in the air

	double drive_[2] = { 0, 0 };
	bool coast_[2] = { true, true };
};
