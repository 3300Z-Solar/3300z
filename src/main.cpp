#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "macro.hpp"
#include <algorithm>
#include <cstdio>



using pros::delay;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// pros::MotorGroup rightMotors({5, 3, -1}, pros::MotorGearset::blue);
// pros::MotorGroup leftMotors({-6, -7, 8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9, 7, -8}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-4, -2, 3}, pros::MotorGearset::blue);
pros::MotorGroup claw({20}, pros::MotorGearset::green);
pros::MotorGroup puhtongue({-5}, pros::MotorGearset::green);
pros::adi::Pneumatics salute('h', false);


pros::Imu imu(14);



// tracking wheels
pros::Rotation horizontalEnc(-15);
pros::Rotation verticalEnc(6);
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
lemlib::ControllerSettings linearController(8, 
                                            0, 
                                            45,
                                            0, 
                                            0, 
                                            10000, 
                                            0, 
                                            10000, 
                                            0
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             13.875, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
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
    
    pros::Task screenTask([&]() {
        char logPath[32] = "/usd/1.json";
        if (pros::usd::is_installed()) {
            int n = 1;
            FILE* file;
            do {
                snprintf(logPath, sizeof(logPath), "/usd/%d.json", n);
                file = fopen(logPath, "r");
                if (file) fclose(file);
                else break;
                n++;
            } while (n < 10000);
        }

        while (true) {
            // print robot location to the brain screen
            // pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position JSON to the terminal
            printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                   chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, pros::millis());
            if (pros::usd::is_installed()) {
                FILE* file = fopen(logPath, "a");
                if (file) {
                    fprintf(file,
                            "{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
                            chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta, pros::millis());
                    fclose(file);
                }
            }
            // delay to save resources
            delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    chassis.moveToPoint(0, 24, 1000000);

}

void opcontrol() {
    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    


    pros::Task mechanismTask([]() {
        int printCounter = 0;
        double liftJogVoltage = 0;
        const double liftJogSlewRate = 170;
        while (true) {

            double liftJogTarget = 0;
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                liftJogTarget = 7000;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                liftJogTarget = -10000;
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
                puhtongue.move_voltage(12000);
            } else {
                puhtongue.brake();
            }

            // if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            //     salute.set_value(true);
            // } else {
            //     salute.set_value(false);
            // }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                claw.move_voltage(12000);
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                claw.move_voltage(-12000);
            } else {
                claw.brake();
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

            if (printCounter % 50 == 0) {
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