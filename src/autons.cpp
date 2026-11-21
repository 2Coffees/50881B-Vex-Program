#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
const int RAM_SPEED = 90;
// ===== Lift / claw =====
// Lift positions are motor degrees measured from fully down (tared at auton start).
// To find yours: move the lift by hand and print dr4b.get_position_all()[0].
const int LIFT_GRAB  = -21;  // TODO measure: height to pick a pin/cup off the floor
const int LIFT_CARRY = -21;  // TODO measure: height for driving with a pin
const int LIFT_SCORE1 = -397; // TODO measure: preload into the empty red goal
const int LIFT_SCORE2 = -597; // TODO measure: pin + cup onto a goal that already has 1 pin (red goal and black goal)

///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// // Move the lift and wait until it gets there (or times out).
void lift_set(int target, int timeout_ms = 1200) {
  dr4b.move_absolute(target, 100);
  uint32_t start = pros::millis();
  while (fabs(dr4b.get_position_all()[0] - target) > 15 &&
         pros::millis() - start < timeout_ms) {
    pros::delay(10);
  }
}

// Wrist: WRIST_IN / WRIST_OUT live in your motors header (see opcontrol_and_motors.cpp).
void wrist_set(int target, int timeout_ms = 800) {
  wrist.move_absolute(target, 100);
  uint32_t start = pros::millis();
  while (fabs(wrist.get_position() - target) > 10 &&
         pros::millis() - start < timeout_ms) {
    pros::delay(10);
  }
}

// Intake rollers: R2 (+127) = roll in, L2 (-127) = roll out, same as your opcontrol.
void claw_close() {
  claw.move(127);
  pros::delay(300);   // TODO tune: time for the rollers to pull the pin in
  claw.move(35);      // light inward power so the rollers keep squeezing the pin while driving
}

void claw_open() {
  claw.move(-127);
  pros::delay(250);   // TODO tune: time for the rollers to spit the pin out
  claw.brake();
}

void grab_pin() {
  wrist_set(WRIST_OUT);   // reach out to the sideways pin
  claw_close();
  wrist_set(WRIST_IN);    // bring it back in to carry
  lift_set(LIFT_CARRY);
}

// Raise to the given score height, release, then drop back to grab height with the claw open.
void score_pin(int height) {
  lift_set(height);
  claw_open();
  lift_set(LIFT_GRAB);
}
// =======================

void Red_Left() {
  chassis.odom_xyt_set(0_in, -65.8_in, 0_deg);   // facing field center, toggle mech on the back

  // Auton only: your brake modes are set in opcontrol, so set them here too
  dr4b.tare_position();                          // lift must start fully down
  wrist.tare_position();                         // wrist must start stowed (in)
  wrist.set_brake_mode(pros::MotorBrake::hold);
  dr4b.set_brake_mode_all(pros::MotorBrake::hold);
  claw.set_brake_mode(pros::MotorBrake::brake);
  claw.move(35);                                 // keep a light grip on the preload
  lift_set(LIFT_CARRY);

  // 1) Ram the toggle twice: forward out, reverse back into it
  for (int i = 0; i < 2; i++) {
    chassis.pid_odom_ptp_set({{0_in, -62.0_in}, fwd, DRIVE_SPEED});
    chassis.pid_wait();
    chassis.pid_odom_ptp_set({{0_in, -67.8_in}, rev, RAM_SPEED});
    chassis.pid_wait();
  }

  // 2) Score preload in the red goal (goal is ~10 in to the left)
  chassis.pid_odom_ptp_set({{-13.9_in, -50.0_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(-80_deg, TURN_SPEED);
  chassis.pid_wait();
  score_pin(LIFT_SCORE1);

  // 3) Grab the yellow pin + cup
  chassis.pid_odom_ptp_set({{-25.1_in, -34.0_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  grab_pin();

  // 4) Score it in the red goal (this point makes the robot spin ~180 to face it)
  chassis.pid_odom_ptp_set({{-25.1_in, -38.8_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  score_pin(LIFT_SCORE2);

  // ---- everything below is the "if we have time" part ----

  // 5) Grab the red/blue pin
  chassis.pid_odom_ptp_set({{0_in, -33.4_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  grab_pin();

  // 6) Score it in the black goal (goal is ~9 in to the right)
  chassis.pid_odom_ptp_set({{14.9_in, -50.0_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(80_deg, TURN_SPEED);
  chassis.pid_wait();
  score_pin(LIFT_SCORE2);
}

void Safe_Auton() {
  chassis.odom_xyt_set(0_in, -65.8_in, 0_deg);   // facing field center, toggle mech on the back

  dr4b.tare_position();                          // lift must start fully down
  wrist.tare_position();                         // wrist must start stowed (in)
  wrist.set_brake_mode(pros::MotorBrake::hold);
  dr4b.set_brake_mode_all(pros::MotorBrake::hold);
  claw.set_brake_mode(pros::MotorBrake::brake);
  claw.move(35);                                 // keep a light grip on the preload
  lift_set(LIFT_CARRY);

  // Ram the toggle twice
  for (int i = 0; i < 2; i++) {
    chassis.pid_odom_ptp_set({{0_in, -62.0_in}, fwd, DRIVE_SPEED});
    chassis.pid_wait();
    chassis.pid_odom_ptp_set({{0_in, -67.8_in}, rev, RAM_SPEED});
    chassis.pid_wait();
  }

  // Score the preload in your alliance goal (~10 in to the left)
  chassis.pid_odom_ptp_set({{-13.9_in, -50.0_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(-80_deg, TURN_SPEED);
  chassis.pid_wait();
  score_pin(LIFT_SCORE1);
}

void Do_Nothing() {}