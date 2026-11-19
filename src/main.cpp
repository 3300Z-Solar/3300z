#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "macro.hpp"
#include "pros/rtos.h"
#include <algorithm>
#include <cstdint>
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

    // pros::Task loggingTask([&]() {
    //     while (true) {
    //         lemlib::Pose pose = chassis.getPose();
    //         pros::lcd::print(0, "X: %.2f", pose.x);         // x
    //         pros::lcd::print(1, "Y: %.2f", pose.y);         // y
    //         pros::lcd::print(2, "Theta: %.2f", pose.theta); // heading
    //         printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
    //                pose.x, pose.y, pose.theta, pros::millis());
    //         delay(50);
    //     }
    // });
}

void disabled() {}

void competition_initialize() {}

void driveToPin(double x, double y, int32_t timeout){
    lemlib::Pose pose = chassis.getPose();
    double dx = x - pose.x;
    double dy = y - pose.y;
    double dist = pose.distance(lemlib::Pose(x,y));
    liftSetTarget(0);
    claw.move_voltage(12000);
    chassis.turnToPoint(x, y, timeout/2);
    chassis.moveToPoint(
        x,
        y,
        timeout/2,
        {.maxSpeed = 64}
    );
    chassis.waitUntilDone();
    chassis.tank(32, 32, true);
}

void goalReset(double goalx, double goaly) {
    chassis.waitUntilDone();
    double theta = lemlib::degToRad(chassis.getPose().theta);

    double x = goalx - DIST_FROM_GOAL * sin(theta);
    double y = goaly - DIST_FROM_GOAL * cos(theta);

    chassis.setPose(x, y, chassis.getPose().theta);
}



void lift_test() {
    pros::Task lift_logging_task([&]() {
        printf("[");
        while (true) {
            // lemlib::Pose pose = chassis.getPose();
            // pros::lcd::print(0, "X: %.2f", pose.x);         // x
            // pros::lcd::print(1, "Y: %.2f", pose.y);         // y
            // pros::lcd::print(2, "Theta: %.2f", pose.theta); // heading
            // // printf("{\"pose\":{\"x\":%.2f,\"y\":%.2f,\"theta\":%.2f},\"t\":%u}\n",
            // //        pose.x, pose.y, pose.theta, pros::millis());
            printf("(%.2u, %.2f),", pros::millis(), liftEncGetPosition());
            delay(20);
        }
    });
    liftControl(6000, 3000);
    // liftMotors.move_voltage(-12000);
    // delay(2000);
    // liftMotors.move_voltage(12000);
    // delay(2000);
    // liftMotors.brake();
}

void josh_far() {

    chassis.setPose(-34, -60, 41);
    // claw.move_voltage(12000); delay(100); claw.move_voltage(-12000); delay(100); claw.move_voltage(12000);
    liftControl(6500, 1000);
    
    
    // --------- score 1 ---------

    // chassis.moveToPoint(-23.5, -47, 9999); chassis.waitUntilDone(); goalReset(-23.5, -47); // move to goal
    chassis.tank(32, 32, true); delay(700); // alternate move to goal

    liftControl(8200, 400); claw.move_voltage(-12000); delay(800); liftControl(6000, 1000); // score
    

    chassis.moveToPoint(-41, -56, 1000, {.forwards = false}); chassis.waitUntilDone(); // go back
    // chassis.moveToPoint(-45, -60, 700, {.forwards = false, .maxSpeed = 63}); // alternate go back

    // --------- score 2 ---------


    
    liftSetTarget(8200);

    claw.move_voltage(12000); chassis.turnToPoint(-48, -46, 700); chassis.moveToPoint(-48, -46, 700, {.maxSpeed = 78}); // intake second stack

    chassis.waitUntilDone(); liftControl(-1000, 1000); claw.move(0); // lift up
    

    // driveToPin(-48, -46, 1400); delay(500); liftControl(-1000, 1000); claw.brake(); // alternative grab pin
    chassis.turnToPoint(-23.5, -47, 700);

    
    chassis.moveToPoint(-31, -47, 1500); chassis.waitUntilDone(); chassis.tank(32, 32, true); goalReset(-23.5, -47); // move to goal

    liftControl(8200, 1000); delay(200); claw.move_voltage(-12000); delay(700); // lift down and let go


    // --------- score 3 ---------

    chassis.moveToPoint(-48, -46, 1000, {.forwards = false});
    driveToPin(-23.5, -23.5, 5000); delay(700); 
    
    chassis.turnToPoint(-23.5, -47, 700);

    liftControl(-1500, 1000);

    chassis.moveToPoint(-23.5, -39.5, 1500); chassis.waitUntilDone(); chassis.tank(32, 32, true); goalReset(-23.5, -47); // move to goal

    liftControl(8200, 1000); delay(200); claw.move_voltage(-12000); delay(700); // lift down and let go

    // --------- toggle ---------

    chassis.moveToPoint(0, -37, 1000, {.forwards = false}); // go back

    chassis.waitUntilDone(); rsalute.set_value(true); claw.move_voltage(0); liftControl(0, 700); // lift down, claw stop, salute up

    chassis.turnToPoint(0, -64, 700); chassis.moveToPoint(0, -64, 1500); // ram into toggle

    chassis.waitUntilDone(); rsalute.set_value(false); chassis.moveToPoint(0, -42, 1000, {.forwards = false}); // toggle

}

void richard_auto() {

    // flip toggle and release banding
    chassis.setPose(-67, 0, 90);
    claw.move_voltage(12000);
    delay(70);
    claw.brake();
    btoggle.move_voltage(-12000);
    delay(400);
    btoggle.brake();
    claw.move_voltage(12000);

    // move to goal and score pin 1
    liftSetTarget(6500);
    chassis.moveToPoint(-60, -19, 1200);
    chassis.waitUntilDone();
    chassis.tank(32, 32, false);
    goalReset(-47, -23.5);
    liftSetTarget(7700);
    delay(400);
    claw.move_voltage(-12000);
    delay(350);
    claw.brake();

    // back out of goal and grab 2
    chassis.moveToPoint(-32, 4, 900, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToPoint(-24.5, -22, 700);
    chassis.moveToPoint(-24.5, -22, 1500, {.maxSpeed = 90});
    liftSetTarget(8200);
    claw.move_voltage(-12000);
    delay(500);
    claw.brake();
    delay(210);
    claw.move_voltage(12000);
    delay(350);

    // move to goal and score pin 2
    liftSetTarget(5500);
    chassis.moveToPoint(-47, -23.5, 1200, {.maxSpeed = 127});
    chassis.waitUntilDone();
    goalReset(-47, -23.5);
    claw.move_voltage(12000);
    liftSetTarget(8200);
    delay(200);
    claw.move_voltage(-12000);
    delay(350);
    claw.brake();

    // back out of goal and grab 3
    chassis.moveToPoint(-29, -29.0, 1000, {.forwards = false});
    liftSetTarget(8200);
    chassis.turnToPoint(-44.5, -43.5, 200);
    chassis.moveToPoint(-44.5, -43.5, 700);
    claw.move_voltage(-12000);
    claw.brake();
    delay(510);
    claw.move_voltage(12000);
    delay(100);
    

    // move to goal and score pin 3
    liftSetTarget(4000);
    delay(200);
    chassis.turnToPoint(-47, -23.5, 700);
    chassis.moveToPoint(-47, -23.5, 1000);
    chassis.waitUntilDone();
    goalReset(-47, -23.5);
    liftSetTarget(8200);
    delay(200);
    claw.move_voltage(-12000);
}
void skills() {

    // flip toggle and release banding
    chassis.setPose(-67, 0, 90);
    claw.move_voltage(12000);
    delay(70);
    claw.brake();
    btoggle.move_voltage(-12000);
    delay(400);
    btoggle.brake();
    claw.move_voltage(12000);

    // move to goal and score pin 1
    liftSetTarget(6500);
    chassis.moveToPoint(-60, -19, 1200);
    chassis.waitUntilDone();
    chassis.tank(32, 32, false);
    goalReset(-47, -23.5);
    liftSetTarget(7700);
    delay(400);
    claw.move_voltage(-12000);
    delay(350);
    claw.brake();

    // back out of goal and grab 2
    chassis.moveToPoint(-32, 4, 900, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToPoint(-24.5, -22, 700);
    chassis.moveToPoint(-24.5, -22, 1500, {.maxSpeed = 90});
    liftSetTarget(8200);
    claw.move_voltage(-12000);
    delay(500);
    claw.brake();
    delay(210);
    claw.move_voltage(12000);
    delay(350);

    // move to goal and score pin 2
    liftSetTarget(5500);
    chassis.moveToPoint(-47, -23.5, 1200, {.maxSpeed = 127});
    chassis.waitUntilDone();
    goalReset(-47, -23.5);
    claw.move_voltage(12000);
    liftSetTarget(8200);
    delay(200);
    claw.move_voltage(-12000);
    delay(350);
    claw.brake();

    // back out of goal and grab 3
    chassis.moveToPoint(-29, -29.0, 1000, {.forwards = false});
    liftSetTarget(8200);
    chassis.turnToPoint(-44.5, -43.5, 200);
    chassis.moveToPoint(-44.5, -43.5, 700);
    claw.move_voltage(-12000);
    claw.brake();
    delay(510);
    claw.move_voltage(12000);
    delay(100);
    

    // move to goal and score pin 3
    liftSetTarget(4000);
    delay(200);
    chassis.turnToPoint(-47, -23.5, 700);
    chassis.moveToPoint(-47, -23.5, 1000);
    chassis.waitUntilDone();
    // chassis.tank(32, 32, true);
    liftSetTarget(8200);
    delay(200);
    goalReset(-47, -23.5);
    claw.move_voltage(-12000);

    // move to matchloader

    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
    chassis.turnToPoint(-68.5, -59, 1000);
    chassis.moveToPoint(-63, -59, 1000);
    claw.move_voltage(12000);
    chassis.moveToPoint(-47, -59, 1000, {.forwards = false});
    liftSetTarget(3000);
    delay(200);
    chassis.turnToPoint(-47, -23.5, 700);
    chassis.moveToPoint(-47, -23.5, 1000);
    chassis.waitUntilDone();
    // chassis.tank(32, 32, true);
    liftSetTarget(7000);
    delay(200);
    goalReset(-47, -23.5);
    claw.move_voltage(-12000);



}

void autonomous() {
    skills();
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
                claw.move_voltage(-10000);
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