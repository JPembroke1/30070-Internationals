package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    GoBildaPinpointDriver pinpoint;
    public void init(@NonNull HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

    }

    public void setPose() {

    }

    public void aimTurret() {
        //WORKING

        double robotX = pinpoint.getPosX(DistanceUnit.CM);
        double robotY = pinpoint.getPosY(DistanceUnit.CM);
        double robotHeading = Math.toRadians(pinpoint.getHeading(AngleUnit.DEGREES));

// Vector to target
        double dx = targetPose.getX() + robotX;
        double dy = targetPose.getY() + robotY;

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


    }
}
