#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
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
// pros::MotorGroup rightMotors({5, 3, -1}, pros::MotorGearset::blue);
// pros::MotorGroup leftMotors({-6, -7, 8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({7, 10, -8}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-4, 3, -2}, pros::MotorGearset::blue);
pros::MotorGroup sidetoggle({-5}, pros::MotorGearset::green);
pros::MotorGroup claw({-16}, pros::MotorGearset::green);
pros::MotorGroup toggle({5}, pros::MotorGearset::green);

// claw piston, ADI port H
pros::adi::Pneumatics salute('h', true);

pros::Imu imu(14);

pros::Distance liftDistance(18); // detects when an object is in the lift
//int CLAW_CLOSE_DISTANCE_MM = 30; // TODO: tune this - claw auto-closes when an object is closer than this



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
    salute.set_value(true);
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize();
    // thread for position logging
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::Task liftTask{liftTaskFn}; // lemlib PID loop for the lift
    // liftControl(liftStates[0]);

    pros::Task loggingTask([&]() {
        while (true) {
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                   chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, pros::millis());
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
void autonomous() {

    
    // liftControl(-1200, 750);
    // delay(750);
    // liftControl(-400, 750);
    // delay(750);
    // liftControl(-500, 750);
    // delay(750);
    // liftControl(-600, 750);
    // delay(750);
    // liftControl(-300, 750);

    chassis.setPose(0, 0, 229);
    liftSetTarget(-500);
    chassis.tank(32, 32, true);
    delay(700);
    goalReset(47, 23.5);
    //score first pin
    liftControl(-82, 400);
    claw.move_voltage(-12000);

    delay(800);
    chassis.moveToPoint(60, 45, 700, {.forwards = false, .maxSpeed = 63});
    // start intaking
    liftSetTarget(0);
    

    chassis.turnToPoint(47,47, 700, {.maxSpeed = 127});
    chassis.waitUntilDone();
    claw.move_voltage(12000);
    delay(200);
    chassis.moveToPoint(47, 48, 800);
    // height for second pin
    chassis.waitUntilDone();
    liftControl(-830, 500);
    claw.move(0);
    

    chassis.moveToPoint(47, 23.5, 1000);
    chassis.waitUntilDone();
    chassis.tank(32, 32, true);
    delay(100);
    goalReset(47, 23.5);
    liftControl(-250, 1000);
    delay(200);
    claw.move_voltage(-12000);
    delay(700);
    //score second pin

    // goalReset(47, 23.5);
    // delay(500);
    chassis.moveToPoint(37, 8.5, 1000, {.forwards = false});
    claw.move_voltage(0);
    liftControl(0, 700);
    // back up
    // claw.move_voltage(12000);
    // chassis.turnToPoint(23.5, 23.5, 900);
    // chassis.moveToPoint(23.5, 24.2, 800, {.maxSpeed = 63});
    // chassis.waitUntilDone();
    // liftControl(-950, 1000);
    // claw.move_voltage(0);
    // //pick up 3rd pin
    // chassis.moveToPoint(47, 23.5, 1500, {.maxSpeed = 127});
    // chassis.waitUntilDone();
    // chassis.tank(-32, -32, true);
    // delay(500);
    // goalReset(47, 23.5);
    // liftControl(-400, 1000);
    // claw.move_voltage(-12000);
    // delay(600);
    // // chassis.moveToPoint(44, 34, 200, {.forwards = false});
    // claw.move_voltage(0);
    //score 3rd
    chassis.moveToPoint(29.5, 0, 300, {.forwards = false});
    salute.set_value(false);
    chassis.moveToPoint(76, 5, 2000, {.forwards = false});
    delay(1700);
    salute.set_value(true);
    chassis.moveToPoint(40, 0, 800);



    // delay(4000);
    // chassis.turnToPoint(47, 23.5, 4000);
    // chassis.tank(32, 32, true);
    // goalReset(47, 23.5);
    // liftMotors.move_voltage(-7000);
    // delay(500);
    // liftMotors.move_voltage(0);
    // claw.move_voltage(-12000);
    // delay(250);
    // claw.move_voltage(0);

    

    // chassis.swingToHeading(-90, lemlib::DriveSide::LEFT, 10000);

    // move 48" forwards
    // chassis.turnToHeading(90, 1000);
    // chassis.turnToHeading(180, 1000);
    // chassis.turnToHeading(270, 1000);
    // chassis.turnToHeading(360, 1000);
    // chassis.turnToHeading(360, 3000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    
}

void opcontrol() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftManual(); // release any PID hold left over from autonomous


    pros::Task mechanismTask([]() {
        int printCounter = 0;
        double liftJogVoltage = 0;
        const double liftJogSlewRate = 170;
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
                salute.set_value(true);
            } else {
                salute.set_value(false);
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


            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                toggle.move_voltage(-12000);
            }
            else {
                toggle.brake();
                toggle.move_voltage(0);
            }

            // static bool objectWasClose = false;
            // bool objectIsClose = liftDistance.get_distance() < CLAW_CLOSE_DISTANCE_MM;
            // if (clawSensorSuppressMs > 0) {
            //     clawSensorSuppressMs -= 10;
            // } else if (objectIsClose && !objectWasClose) {
            //     claw.set_value(false);
            //     delay(200);
            //     liftControl(liftStates[1]);
            // }
            // objectWasClose = objectIsClose;


            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                // liftControl(liftStates[0]);
                delay(100);
                // liftControl(liftStates[0]);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                // liftControl(liftStates[1]);
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