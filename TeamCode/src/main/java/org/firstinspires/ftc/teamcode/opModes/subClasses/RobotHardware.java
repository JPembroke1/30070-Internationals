package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class RobotHardware {

    public Servo hoodServo = null;
    public Servo blockServo = null;

    public static double hoodSlope = -0.002959;
    public static double hoodIntercept = 0.552438;

    public static double hoodVelocityFactor = 0.05;

    public static double hoodMin = 0.16;
    public static double hoodMax = 0.55;

    public static double hoodDeadband = 0.001;
    public static double hoodMaxStep = 0.02;

    public static boolean useDistanceSmoothing = true;
    public static double distanceAlpha = 0.4;

    public static double lastCommandedHood = 0.5;
    public static double filteredDistanceCM = 0.0;
    public static boolean hoodInitialised = false;

    public RobotHardware(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        blockServo.setDirection(Servo.Direction.REVERSE);
    }

    public void close_range() {
        setHoodPosition(RobotSettings.close);
    }

    public void mid_range() {
        setHoodPosition(RobotSettings.mid);
    }

    public void long_range() {
        setHoodPosition(RobotSettings.far);
    }

    public void setHoodPosition(double targetPosition) {
        double clipped = clamp(targetPosition, hoodMin, hoodMax);

        if (!hoodInitialised) {
            lastCommandedHood = clipped;
            hoodServo.setPosition(clipped);
            hoodInitialised = true;
            return;
        }

        if (Math.abs(clipped - lastCommandedHood) < hoodDeadband) {
            return;
        }

        double delta = clipped - lastCommandedHood;
        delta = clamp(delta, -hoodMaxStep, hoodMaxStep);

        double newPosition = lastCommandedHood + delta;
        newPosition = clamp(newPosition, hoodMin, hoodMax);

        hoodServo.setPosition(newPosition);
        lastCommandedHood = newPosition;
    }

    public void linearHoodRegression(double distanceCM) {
        double workingDistanceCM = filterDistance(distanceCM);
        double velocityComp = calculateVelocityCompensation();
        double position = (hoodSlope * workingDistanceCM) + hoodIntercept + velocityComp;
        setHoodPosition(position);
    }

    public void linearHoodRegression(double formula, double distanceCM, double yIntercept) {
        double workingDistanceCM = filterDistance(distanceCM);
        double velocityComp = calculateVelocityCompensation();
        double position = (formula * workingDistanceCM) + yIntercept + velocityComp;
        setHoodPosition(position);
    }

    private double calculateVelocityCompensation() {
        if (Outtake.target <= 0) {
            return 0.0;
        }

        double tpsRatio = Outtake.currentTPS / Outtake.target;
        tpsRatio = clamp(tpsRatio, 0.0, 1.2);

        double velocityComp = (1.0 - tpsRatio) * hoodVelocityFactor;
        return Math.max(0.0, velocityComp);
    }

    private double filterDistance(double distanceCM) {
        if (!useDistanceSmoothing) {
            filteredDistanceCM = distanceCM;
            return distanceCM;
        }

        if (!hoodInitialised) {
            filteredDistanceCM = distanceCM;
        } else {
            filteredDistanceCM = (distanceAlpha * distanceCM) + ((1.0 - distanceAlpha) * filteredDistanceCM);
        }

        return filteredDistanceCM;
    }

    public void block() {
        blockServo.setPosition(RobotSettings.block);
    }

    public void release() {
        blockServo.setPosition(RobotSettings.release);
    }

    public void reset_all() {
        hoodInitialised = false;
        filteredDistanceCM = 0.0;
        lastCommandedHood = clamp(RobotSettings.close, hoodMin, hoodMax);

        hoodServo.setPosition(lastCommandedHood);
        blockServo.setPosition(RobotSettings.block);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}