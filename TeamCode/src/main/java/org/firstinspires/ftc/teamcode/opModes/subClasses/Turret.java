package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opModes.InternationalsOfTheTeleops;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {
    GoBildaPinpointDriver pinpoint;

    public Servo rotationalTurretServo = null;

    public static double kP = 0.1;
    public static Pose targetPose = new Pose(0, 144, 0);

    public static double dx = 0;
    public static double dy = 0;
    public static double robotX = 0;
    public static double robotY = 0;
    public static double targetWorldAngle = 0;

    public void init(@NonNull HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

    }

    public void setPose(Pose pose) {
        targetPose = pose;
    }

    public void centre() {
        rotationalTurretServo.setPosition(0.5);
    }

//    public void aimTurret() {
//      //WORKING
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//        double robotY = pinpoint.getPosX(DistanceUnit.CM);
//        double robotX = pinpoint.getPosY(DistanceUnit.CM);
//        double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));
//
//
//
//// Vector to target
//        double dx = targetPose.getX() - robotX;
//        double dy = targetPose.getY() - robotY;
//
//// World-space angle to target
//        double targetHeading = Math.atan2(dy, dx);
//
//// Current turret angle (0 → 180 deg)
//       double turretAngle = Math.toRadians((-rotationalTurretServo.getPosition() * 180.0) + 90);
//        //double turretAngle = Math.toRadians(rotationalTurretServo.getPosition() * 180);
//        //double turretAngle = Math.toRadians((rotationalTurretServo.getPosition() - 0.5) * 180);
//// Turret world direction
//        double turretWorldHeading = robotHeading + turretAngle;
//
//// Angle error
//        double error = targetHeading - turretWorldHeading;
//        error = Math.atan2(Math.sin(error), Math.cos(error)); // wrap
//
//// Apply proportional correction
//        double newTurretAngle = turretAngle + error * kP;
//
//// Clamp to physical limits (0° → 180°)
//        //newTurretAngle = Math.max(0, Math.min(Math.PI, newTurretAngle));
//        newTurretAngle = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, newTurretAngle));
//// Convert back to servo position
//        double servoPosition = Math.toDegrees((newTurretAngle) + 90) / 180;
//        //double servoPosition = (90 - Math.toDegrees(newTurretAngle)) / 180;
//
//        double epsilon = 0.001;
//
//        boolean atMin = servoPosition <= 0.0 + epsilon;
//        boolean atMax = servoPosition >= 1.0 - epsilon;
//
//        boolean pushingIntoLimit = (atMin && error < 0) || (atMax && error > 0);
//
//        if (!pushingIntoLimit) {
//            rotationalTurretServo.setPosition(servoPosition);
//        }
//
//        //Velocity Compensation
//        double velocity = Math.sqrt((pinpoint.getVelX(DistanceUnit.CM) * pinpoint.getVelX(DistanceUnit.CM)) + ((pinpoint.getVelY(DistanceUnit.CM) * pinpoint.getVelY(DistanceUnit.CM))));
//        double acclSpeed = 0;
//        double dcclSpeed = 0;
//
//
//
//
//// Deadband
//                //if (Math.abs(error) > Math.toRadians(2)) {
//                //    rotationalTurretServo.setPosition(servoPosition);
//                //}
//
//
//    }

public void aimTurret() {
    // --- Robot pose in world space ---
    robotX = -pinpoint.getPosY(DistanceUnit.CM);   // right = +X
    robotY = -pinpoint.getPosX(DistanceUnit.CM);   // forward = +Y
    double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

    // --- Vector from robot to target (world space) ---
    dx = targetPose.getX() - robotX;
    dy = targetPose.getY() - robotY;

    // --- World-space angle to target ---
    targetWorldAngle = Math.atan2(dx, dy);

    // --- Angle to target relative to robot heading ---
    // 0 = straight ahead, positive = left, negative = right
    double relativeAngle = targetWorldAngle - robotHeading;
    relativeAngle = Math.atan2(Math.sin(relativeAngle), Math.cos(relativeAngle)); // wrap to [-π, π]

    // --- Convert relative angle to servo position ---
    // Servo: 0.0 = 90° right (-π/2), 0.5 = straight ahead (0°), 1.0 = 90° left (+π/2)
    // servoPosition = 0.5 - (relativeAngle / π)
    double desiredServoPosition = 0.5 - (relativeAngle / Math.PI);

    // --- Read current servo position and compute error ---
    double currentServoPosition = rotationalTurretServo.getPosition();
    double servoError = desiredServoPosition - currentServoPosition;

    // --- Proportional correction ---
    double correction = servoError * kP;

    // --- Max step clamp (prevents wild overshoot) ---
    double maxStep = 0.05; // tune this
    correction = Math.max(-maxStep, Math.min(maxStep, correction));

    double newServoPosition = currentServoPosition + correction;

    // --- Clamp to physical limits [0.0, 1.0] ---
    newServoPosition = Math.max(0.0, Math.min(1.0, newServoPosition));

    // --- Limit guard: don't push into a hard stop ---
    boolean atMin = newServoPosition <= 0.001;
    boolean atMax = newServoPosition >= 0.999;
    boolean pushingIntoLimit = (atMin && servoError < 0) || (atMax && servoError > 0);

    if (!pushingIntoLimit) {
        rotationalTurretServo.setPosition(newServoPosition);
    }

    // --- Velocity compensation (placeholder) ---
    double velX = pinpoint.getVelX(DistanceUnit.CM);
    double velY = pinpoint.getVelY(DistanceUnit.CM);
    double velocity = Math.sqrt(velX * velX + velY * velY); // fixed magnitude calc
    // TODO: use velocity to lead the target if needed
}
}
