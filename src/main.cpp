#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "macro.hpp"
#include "pros/rtos.h"
#include <algorithm>
#include <cstdio>



using pros::delay;

// flip to true when liftStates need re-measuring/re-tuning; shows lift
// position on the controller instead of lift motor temperature
constexpr bool SHOW_LIFT_POS_DEBUG = false;
const double DIST_FROM_GOAL = 7.5;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// old (sides swapped + reversed, needed -leftY in driver to work):
//pros::MotorGroup rightMotors({4, -3, 2}, pros::MotorGearset::blue);
//pros::MotorGroup leftMotors({-7, -10, 8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({7, 10, -8}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-4, 3, -2}, pros::MotorGearset::blue);
pros::MotorGroup sidetoggle({-5}, pros::MotorGearset::green);
pros::MotorGroup claw({-16}, pros::MotorGearset::green);
pros::MotorGroup btoggle({13}, pros::MotorGearset::green);

// claw piston, ADI port H
pros::adi::Pneumatics rsalute('g', true);
pros::adi::Pneumatics lsalute('h', true);

pros::Imu imu(14);



// tracking wheels
pros::Rotation horizontalEnc(15);
pros::Rotation verticalEnc(6);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12,
                              lemlib::Omniwheel::NEW_325,
                              450,
                              2 
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, 
                                            0, 
                                            58,
                                            3, 
                                            0.001, 
                                            100, 
                                            0.001, 
                                            500, 
                                            0 
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             13.875, // derivative gain (kD)
                                             3, // anti windup
                                             0.001, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             0.001, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control — exponential
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// no steerCurve object needed — steering will use lemlib::defaultDriveCurve

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        linearController,
                        angularController,
                        sensors,
                        &throttleCurve,
                        &lemlib::defaultDriveCurve // linear steering, no expo shaping
);

void initialize() {
    rsalute.set_value(false);
    lsalute.set_value(true);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize();
    // thread for position logging
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task liftTask{liftTaskFn}; // lemlib PID loop for the lift
    // liftControl(liftStates[0]);

    pros::Task loggingTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f", pose.x);         // x
            pros::lcd::print(1, "Y: %.2f", pose.y);         // y
            pros::lcd::print(2, "Theta: %.2f", pose.theta); // heading
            printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                   pose.x, pose.y, pose.theta, pros::millis());
            delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}
void goalReset(double goalx, double goaly) {
    double theta = lemlib::degToRad(chassis.getPose().theta);

    double x = goalx - DIST_FROM_GOAL * sin(theta);
    double y = goaly - DIST_FROM_GOAL * cos(theta);

    chassis.setPose(x, y, chassis.getPose().theta);
}

void josh_far() {

    chassis.setPose(-34, -60, 41);
    claw.move_voltage(12000); delay(100); claw.move_voltage(-12000); delay(100); claw.move_voltage(12000);
    liftControl(-700, 1000);
    
    
    // --------- score 1 ---------

    // chassis.moveToPoint(-23.5, -47, 9999); chassis.waitUntilDone(); goalReset(-23.5, -47); // move to goal
    chassis.tank(32, 32, true); delay(700); // alternate move to goal

    liftControl(-82, 400); claw.move_voltage(-12000); delay(800); liftControl(-700, 1000); // score
    

    chassis.moveToPoint(-41, -56, 1000, {.forwards = false}); chassis.waitUntilDone(); // go back
    // chassis.moveToPoint(-45, -60, 700, {.forwards = false, .maxSpeed = 63}); // alternate go back

    // --------- score 2 ---------

    liftSetTarget(0);

    claw.move_voltage(12000); chassis.turnToPoint(-48, -46, 700); chassis.moveToPoint(-48, -46, 700, {.maxSpeed = 78}); // intake second stack

    chassis.waitUntilDone(); liftControl(-1000, 1000); claw.move(0); // lift up
    
    chassis.moveToPoint(-23.5, -47, 1500); chassis.waitUntilDone(); goalReset(-23.5, -47); // move to goal

    liftControl(0, 1000); delay(200); claw.move_voltage(-12000); delay(700); // lift down and let go

    // --------- toggle ---------

    chassis.moveToPoint(0, -37, 1000, {.forwards = false}); // go back

    chassis.waitUntilDone(); rsalute.set_value(true); claw.move_voltage(0); liftControl(0, 700); // lift down, claw stop, salute up

    chassis.turnToPoint(0, -64, 700); chassis.moveToPoint(0, -64, 1500); // ram into toggle

    chassis.waitUntilDone(); rsalute.set_value(false); chassis.moveToPoint(0, -42, 1000, {.forwards = false}); // toggle

}

void autonomous() {
chassis.setPose(0, 0, 0);
claw.move_voltage(12000);
delay(90);
chassis.moveToPoint(0, 1, 500);
claw.brake();
chassis.waitUntilDone();
btoggle.move_voltage(-12000);
delay(500);
btoggle.brake();
claw.move_voltage(12000);
liftSetTarget(-630);
chassis.moveToPoint(15.5, 10.3, 900, {.maxSpeed = 127});

chassis.waitUntilDone();
liftSetTarget(400);
delay(400);
claw.move_voltage(-12000);
delay(500);
claw.brake();

chassis.moveToPoint(-2, 12, 500, {.forwards=false});
chassis.waitUntilDone();
chassis.turnToHeading(52, 500);
chassis.moveToPoint(18.2, 24.2, 1000, {.maxSpeed = 127});
claw.move_voltage(-12000);
delay(550);
claw.brake();
delay(50);
claw.move_voltage(12000);
delay(650);
liftSetTarget(-850);
chassis.turnToHeading(170, 800);
chassis.moveToPoint(24.2, 11.4, 700, {.maxSpeed = 120});
chassis.waitUntilDone();
claw.move_voltage(12000);
liftSetTarget(-125);
delay(200);
claw.move_voltage(-12000);
delay(500);
claw.brake();
chassis.moveToPoint(29, 22, 1000, {.forwards = false,.maxSpeed = 127});
chassis.turnToHeading(135, 800);
liftSetTarget(0);
chassis.moveToPoint(39, 10, 700, {.maxSpeed = 120});
chassis.waitUntilDone();
claw.move_voltage(12000);
delay(650);
claw.brake();
liftSetTarget(-1250);
chassis.turnToHeading(270, 800);
chassis.moveToPoint(26.9, 3.6, 700, {.maxSpeed = 120});
chassis.waitUntilDone();
claw.move_voltage(12000);
liftSetTarget(-125);
delay(200);
claw.move_voltage(-2000);
}

void opcontrol() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftManual(); // release any PID hold left over from autonomous


    pros::Task mechanismTask([]() {
        int printCounter = 0;
        double liftJogVoltage = 0;
        const double liftJogSlewRate = 500;
        const double liftJogKickVoltage = 5000; // initial jump so the lift doesn't crawl before ramping up
        while (true) {

            double liftJogTarget = 0;
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftJogTarget = 6000;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftJogTarget = -12000;
            }

            static bool wasJogging = false;
            bool isJogging = (liftJogTarget != 0);
            if (isJogging) {
                if (!wasJogging) {
                    liftJogVoltage = std::clamp(liftJogTarget, -liftJogKickVoltage, liftJogKickVoltage);
                }
                if (liftJogVoltage < liftJogTarget) {
                    liftJogVoltage = std::min(liftJogVoltage + liftJogSlewRate, liftJogTarget);
                } else if (liftJogVoltage > liftJogTarget) {
                    liftJogVoltage = std::max(liftJogVoltage - liftJogSlewRate, liftJogTarget);
                }
                liftMotors.move_voltage((int) liftJogVoltage);
            } else if (wasJogging) {
                liftMotors.brake();
                liftJogVoltage = 0;
            }
            wasJogging = isJogging;


if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                rsalute.set_value(true);
            } else {
                rsalute.set_value(false);
            }

         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                lsalute.set_value(false);
            } else {
                lsalute.set_value(true);
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                btoggle.move_voltage(-12000);
            } else {
                btoggle.brake();
                btoggle.move_voltage(0);
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                claw.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                claw.move_voltage(-5000);
            }
            else {
                claw.brake();
                claw.move_voltage(0);
            }

            int liftPosDeg = (int) liftGetPosition();
            if (printCounter % 50 == 0) {
                if (SHOW_LIFT_POS_DEBUG) {
                    controller.print(0, 0, "lift pos: %d", liftPosDeg);
                } else {
                    int liftHottest = (int) std::max(liftMotors.get_temperature(0), liftMotors.get_temperature(1));
                    double driveHottestD = leftMotors.get_temperature(0);
                    driveHottestD = std::max(driveHottestD, leftMotors.get_temperature(1));
                    driveHottestD = std::max(driveHottestD, leftMotors.get_temperature(2));
                    driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(0));
                    driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(1));
                    driveHottestD = std::max(driveHottestD, rightMotors.get_temperature(2));
                    int driveHottest = (int) driveHottestD;
                    controller.print(0, 0, "L:%dC,D:%dC", liftHottest, driveHottest);
                }
            }
            printCounter++;

            delay(10);
        }
    });

    // drivetrain loop — runs independently at a steady 10ms cadence
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);
        delay(10);
    }
}