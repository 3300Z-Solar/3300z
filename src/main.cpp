#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "macro.hpp"
#include <algorithm>


using pros::delay;

// flip to true when liftStates need re-measuring/re-tuning; shows lift
// position on the controller instead of lift motor temperature
constexpr bool SHOW_LIFT_POS_DEBUG = false;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({5, 3, -1}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-6, -7, 8}, pros::MotorGearset::blue);


// claw piston, ADI port H
pros::adi::Pneumatics claw('a', false);
pros::adi::Pneumatics salute('b', false);
pros::adi::Pneumatics taiwan('c', false);

pros::Imu imu(20);

pros::Distance liftDistance(18); // detects when an object is in the lift
int CLAW_CLOSE_DISTANCE_MM = 30; // TODO: tune this - claw auto-closes when an object is closer than this



// tracking wheels
pros::Rotation horizontalEnc(9);
pros::Rotation verticalEnc(-10);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0.38);
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
lemlib::ControllerSettings linearController(14, 
                                            0, 
                                            126,
                                            3, 
                                            1, 
                                            100, 
                                            3, 
                                            500, 
                                            94 
);

// angular motion controller
lemlib::ControllerSettings angularController(2.3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             28, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
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
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // thread for brain screen and position logging
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    liftControl(liftStates[0]);
    
    pros::Task screenTask([&]() {
        
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position JSON to the terminal
            printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                   chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, pros::millis());
            // delay to save resources
            delay(100);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
chassis.setPose({0,0,0});
chassis.moveToPoint(0, 10, 5000);

}

void opcontrol() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


    pros::Task mechanismTask([]() {
        int printCounter = 0;
        double liftJogVoltage = 0;
        const double liftJogSlewRate = 170;
        while (true) {

            double liftJogTarget = 0;
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftJogTarget = 11000;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftJogTarget = -8000;
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


            static double clawSensorSuppressMs = 0;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                claw.toggle();
                clawSensorSuppressMs = 3000;
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                salute.set_value(true);
            } else {
                salute.set_value(false);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                taiwan.toggle();
            }

            static bool objectWasClose = false;
            bool objectIsClose = liftDistance.get_distance() < CLAW_CLOSE_DISTANCE_MM;
            if (clawSensorSuppressMs > 0) {
                clawSensorSuppressMs -= 10;
            } else if (objectIsClose && !objectWasClose) {
                claw.set_value(true);
                delay(200);
                liftControl(liftStates[1]);
            }
            objectWasClose = objectIsClose;


            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
                liftControl(liftStates[0]);
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                liftControl(liftStates[1]);
            }

            int liftPosDeg = liftRot.get_position() / 100;
            static int lastPrintedPos = -999999;
            if (printCounter % 50 == 0 && liftPosDeg != lastPrintedPos) {
                pros::lcd::print(3, "lift pos: %d", liftPosDeg);
                lastPrintedPos = liftPosDeg;
            }
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