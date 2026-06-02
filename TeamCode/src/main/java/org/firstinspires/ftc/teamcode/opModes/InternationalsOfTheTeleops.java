/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Configurable
@TeleOp(name="InternationalTelopsOfTheAwesomeness", group="Linear OpMode")
public class InternationalsOfTheTeleops extends LinearOpMode {

    private static double TARGET_SPEED = 2000;
    private static double servoDir = 0.5;
    private static double hoodPos = 0.5;
    private static double blockPos = 0.2;

    public static double target = 0;
    public static double currentTPS = 0;
    public static double originalP = 0.002;
    public static double P = 20;
    public static double I = 0.0;
    public static double D = 0;
    public static double F = 20;
    public static double kP = 0.3;
    public static double mulitplier = 0.01;
    public static double multiplierAmount = 0;
    public static double offsetX = 0;
    public static double offsetY = 0;
    public static boolean goalAimedAt = false; //False = Blue, True = Red
    public static double formulaResult = 2.714286;
    public static double yIntercept = 1199.999933;
    public static double distance = 0;
    public static double formulaResultHood = -0.00098;
    public static double yInterceptHood = 0.594967;
    public static boolean ActivatedShoot = false;

    public static double shootTypeToggle = 2;
    public static double thresholdValue = 0;
    public static boolean shooting = false;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private DcMotor intakeMotorFront = null;
    private DcMotor intakeMotorBack = null;

    private Servo rotationalTurretServo = null;
    private Servo hoodServo = null;
    private Servo blockServo = null;
    private DcMotorEx outtakeMotor1 = null;
    private DcMotorEx outtakeMotor2 = null;
    public static Pose targetPose = new Pose(0, 144, 0);

    GoBildaPinpointDriver pinpoint;

    public PIDController controller;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRight");

        intakeMotorFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeMotorBack = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtakeLeft");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeRight");

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);

        boolean aiming = false;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotationalTurretServo.setPosition(servoDir);
        blockServo.setDirection(Servo.Direction.REVERSE);

        controller = new PIDController(P, I, D);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM,0,0,AngleUnit.DEGREES, 0));



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            updatePIDF();

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x * 0.9;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            if (gamepad1.right_bumper) {
                intakeMotorFront.setPower(1);
                intakeMotorBack.setPower(1);
            } else if (gamepad1.right_trigger > 0.6) {
                if (ActivatedShoot && currentTPS > 1500) {
                    blockServo.setPosition(0.03);
                    intakeMotorFront.setPower(0.7);
                    intakeMotorBack.setPower(0.9);
                }
            } else {
                blockServo.setPosition(0.2);
                intakeMotorFront.setPower(0);
                intakeMotorBack.setPower(0);
                //target = 850;
            }

            /*if (gamepad1.left_bumper) {
                hoodPos -= 0.001;
            }

            if (gamepad1.left_trigger > 0.3) {

            } */

            /* if (gamepad1.square) {
                hoodPos = 0.3;
                TARGET_SPEED = 800;
                target = TARGET_SPEED;
            } */

            if (gamepad1.cross) {
                ActivatedShoot = true;
                hoodPos = 0.3; //0.4
                TARGET_SPEED = 850; //800
                target = TARGET_SPEED;
            } else if (gamepad1.triangle) {
                ActivatedShoot = false;
                target = 0;
                hoodPos = 0.5;
            }

            if (servoDir < 0) {
                servoDir = 0;
            }

            if (servoDir > 1) {
                servoDir = 1;
            }

            if (hoodPos > 0.5) {
                hoodPos = 0.5;
            }

            if (hoodPos < 0) {
                hoodPos = 0;
            }

            if (gamepad1.circle) {
                hoodPos = 0.16;
                TARGET_SPEED = 1700;
                target = TARGET_SPEED;
            }

            if (gamepad1.dpadUpWasPressed()) {
                shooting = !shooting;
            }

            if (shooting == false && shootTypeToggle == 1) {
                TARGET_SPEED = formulaResult * distance + yIntercept; //LINEAR REGRESSION
                target = TARGET_SPEED;
                hoodPos = formulaResultHood * distance + yInterceptHood;
            } else if (shooting == true && shootTypeToggle == 0) {
                TARGET_SPEED = thresholdValue; //THRESH HOLDS
                target = TARGET_SPEED;
            }

           /* if (gamepad1.triangle) {
                target = 0;
                hoodPos = 0.5;
            } */

            if (gamepad1.dpadRightWasPressed()) {
                if (aiming == false) {
                    aiming = true;
                } else if (aiming == true) {
                    aiming = false;
                }
            }


            if (aiming) {
                //WORKING

                double robotX = pinpoint.getPosX(DistanceUnit.CM);
                double robotY = pinpoint.getPosY(DistanceUnit.CM);
                double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

// Vector to target
                double dx = targetPose.getX() + offsetX - robotX;
                double dy = targetPose.getY() + offsetY - robotY;

// World-space angle to target
                double targetHeading = Math.atan2(-dy, dx);

// Current turret angle (0 → 180 deg)
                double turretAngle = Math.toRadians(rotationalTurretServo.getPosition() * 180.0);

// Turret world direction
                double turretWorldHeading = robotHeading + turretAngle;

// Angle error
                double error = targetHeading - turretWorldHeading;
                error = Math.atan2(Math.sin(error), Math.cos(error)); // wrap

// --- tuning ---
                kP = 0.6;

// Apply proportional correction
                double newTurretAngle = turretAngle + error * kP;

// Clamp to physical limits (0° → 180°)
                newTurretAngle = Math.max(0, Math.min(Math.PI, newTurretAngle));

// Convert back to servo position
                double servoPosition = newTurretAngle / Math.PI;

                double epsilon = 0.001;

                boolean atMin = servoPosition <= 0.0 + epsilon;
                boolean atMax = servoPosition >= 1.0 - epsilon;

                boolean pushingIntoLimit = (atMin && error < 0) || (atMax && error > 0);

                if (!pushingIntoLimit) {
                    rotationalTurretServo.setPosition(servoPosition);
                }

                //Velocity Compensation
                double velocity = Math.sqrt((pinpoint.getVelX(DistanceUnit.CM) * pinpoint.getVelX(DistanceUnit.CM)) * ((pinpoint.getVelY(DistanceUnit.CM) * pinpoint.getVelY(DistanceUnit.CM))));
                double acclSpeed = 0;
                double dcclSpeed = 0;



// Deadband
              /*  if (Math.abs(error) > Math.toRadians(1)) {
                    rotationalTurretServo.setPosition(servoPosition);
                } */

            } else if (aiming == false) {
                rotationalTurretServo.setPosition(servoDir);
            }

            if (gamepad1.dpad_left) {
                targetPose = new Pose(pinpoint.getPosX(DistanceUnit.CM), pinpoint.getPosY(DistanceUnit.CM), Math.toRadians(rotationalTurretServo.getPosition() * 170));
            }

            if (gamepad1.right_stick_button) {
                hoodPos += 0.01;
            }

            if (gamepad1.left_stick_button) {
                hoodPos -= 0.01;
            }

           /* if (gamepad1.dpad_down) {
                pinpoint.setHeading(0, AngleUnit.DEGREES);
            }

         if (gamepad1.dpadUpWasPressed()) {
                if (goalAimedAt == false) {
                    goalAimedAt = true;
                    offsetX = 0;
                    offsetY = 20;
                } else if (goalAimedAt == true) {
                    goalAimedAt = false;
                    offsetX = 0;
                    offsetY = 0;
                }
            } */

            if (gamepad1.leftBumperWasPressed()) {
                if (shootTypeToggle == 0) {
                    shootTypeToggle = 1;
                } else if (shootTypeToggle == 1) {
                    shootTypeToggle = 2;
                } else if (shootTypeToggle == 2) {
                    shootTypeToggle = 0;
                }
            }

            if (shootTypeToggle == 0) {
                if (distance > 0 && distance < 30) {
                    thresholdValue = 1300;
                    hoodPos = 0.5;
                } else if (distance > 30 && distance < 60) {
                    thresholdValue = 1350;
                    hoodPos = 0.5;
                } else if (distance > 60 && distance < 90) {
                    thresholdValue = 1400;
                    hoodPos = 0.5;
                } else if (distance > 90 && distance < 120) {
                    thresholdValue = 1500;
                    hoodPos = 0.49;
                } else if (distance > 120 && distance < 150) {
                    thresholdValue = 1600;
                    hoodPos = 48;
                } else if (distance > 150 && distance < 180) {
                    thresholdValue = 1700;
                    hoodPos = 0.47;
                } else if (distance > 180) {
                    TARGET_SPEED = formulaResult * distance + yIntercept; //LINEAR REGRESSION
                    target = TARGET_SPEED;
                    hoodPos = formulaResultHood * distance + yInterceptHood;
                }
            }

            hoodServo.setPosition(hoodPos);

            double robotXD = pinpoint.getPosX(DistanceUnit.CM);
            double robotYD = pinpoint.getPosY(DistanceUnit.CM);

            double dx = targetPose.getX() + offsetX - robotXD;
            double dy = targetPose.getY() + offsetY - robotYD;

            distance = Math.sqrt((dx * dx) + (dy * dy));

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            pinpoint.update();


            telemetry.addData("TurretRotationServo", "ServoDirection: " + rotationalTurretServo.getPosition());
            telemetry.addData("ServoDir", servoDir);
            telemetry.addData("response", controller.calculate(outtakeMotor1.getVelocity(), target));
            telemetry.addData("blockServo", blockServo.getPosition());
            telemetry.addData("TurretHeading", Math.toRadians(rotationalTurretServo.getPosition() * 170));
            telemetry.addData("targetPose", targetPose);
            telemetry.addData("GoalAim", goalAimedAt);
            telemetry.addData("Distance: ", distance);
            telemetry.addData("ShootToggleType: ", shootTypeToggle);
            telemetry.addData("hoodAngle", hoodPos);
            telemetry.addData("power", target);
            telemetry.addData("P", P);
            telemetry.addData("Velocity1", outtakeMotor1.getVelocity());
            telemetry.addData("Velocity2", outtakeMotor2.getVelocity());
            telemetry.update();
        }
    }
    /*private void updatePIDF() {

        controller.setPIDF(P, I, D, F);

        if (target == 0) {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            return;
        }



        double currentTPS = outtakeMotor1.getVelocity();

        double error = target - currentTPS;

        if (Math.abs(error) < 20) {
            error = 0;
        }
        //double pid = controller.calculate(currentTPS, target);
        double pid = (error * P);
        double ff = target * F;

        double response = pid + ff;
        //double response = controller.calculate(currentTPS, target);

        //response = Math.max(-1, Math.min(1, response));
        // Overspeed protection
        if (currentTPS > target) {

            // Hold power instead of accelerating harder
            response -= 0.05;

            // Optional hard lock
            response = Math.min(response, target * F);
        }

// Clamp
        response = Math.max(0, Math.min(1, response));

        outtakeMotor1.setVelocity(response);
        outtakeMotor2.setVelocity(response);

        //P = P + mulitplier;
    } */

    private void updatePIDF() {

        if (target == 0) {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
            return;
        }

        currentTPS = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2;

        if (outtakeMotor1.getVelocity() < 100) {
            currentTPS = outtakeMotor2.getVelocity();
        } else if (outtakeMotor2.getVelocity() < 100) {
            currentTPS = outtakeMotor1.getVelocity();
        } else if (outtakeMotor1.getVelocity() < 100 && outtakeMotor2.getVelocity() < 100 && ActivatedShoot) {
            outtakeMotor1.setPower(0.5);
            outtakeMotor2.setPower(0.5);
        } else {
            currentTPS = (outtakeMotor1.getVelocity() + outtakeMotor2.getVelocity()) / 2;
        }

        double error = target - currentTPS;

        // Deadband
        if (Math.abs(error) < 20) {
            error = 0;
        }

        // Feedforward
        double ff = target * F;

        // Simple proportional correction
        double pid = error * P;

        double response = ff + pid;

        // Overspeed cap
        if (currentTPS > target) {
            response = Math.min(response, ff);
        }

        // Clamp motor power
        response = Math.max(0, Math.min(1, response));

        outtakeMotor1.setPower(response);
        outtakeMotor2.setPower(response);
    }
}
