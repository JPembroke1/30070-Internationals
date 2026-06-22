package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class RobotHardware {

    public Servo hoodServo;
    public Servo blockServo;

    public static double hoodSlope = -0.00365;
    public static double hoodIntercept = 0.8;
    public static double hoodVelocityFactor = 0.9;

    public static double farZoneHoodSlope = -0.00365;
    public static double farZoneHoodIntercept = 0.9;
    public static double farZoneHoodVelocityFactor = 1.5;

    public static double hoodMin = 0.3;
    public static double hoodMax = 0.9;

    public static double hoodDeadband = 0.001;
    public static double hoodMaxStep = 0.04;

    public static boolean useDistanceSmoothing = false;
    public static double distanceAlpha = 0.4;

    public static double lastCommandedHood = 0.20;
    public static double filteredDistanceCM = 0.0;

    public static boolean hoodInitialised = false;
    public static boolean distanceFilterInitialised = false;

    public static double lastRawHoodTarget = 0.0;
    public static double lastClippedHoodTarget = 0.0;

    public RobotHardware(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        blockServo.setDirection(Servo.Direction.REVERSE);

        reset_all();
    }

    public void setHoodPosition(double targetPosition) {
        double clipped = clamp(targetPosition, hoodMin, hoodMax);

        lastRawHoodTarget = targetPosition;
        lastClippedHoodTarget = clipped;

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
        double velocityComp = calculateVelocityCompensation(hoodVelocityFactor);

        double position =
                (hoodSlope * workingDistanceCM)
                        + hoodIntercept
                        + velocityComp;

        setHoodPosition(position);
    }

    public void farZoneHoodRegression(double distanceCM) {
        double workingDistanceCM = filterDistance(distanceCM);
        double velocityComp = calculateVelocityCompensation(farZoneHoodVelocityFactor);

        double position =
                (farZoneHoodSlope * workingDistanceCM)
                        + farZoneHoodIntercept
                        + velocityComp;

        setHoodPosition(position);
    }

    private double calculateVelocityCompensation(double velocityFactor) {
        if (Outtake.target <= 0.0) {
            return 0.0;
        }

        double tpsRatio = Outtake.currentTPS / Outtake.target;
        tpsRatio = clamp(tpsRatio, 0.0, 1.2);

        double velocityComp = (1.0 - tpsRatio) * velocityFactor;

        return Math.max(0.0, velocityComp);
    }

    private double filterDistance(double distanceCM) {
        if (!useDistanceSmoothing) {
            filteredDistanceCM = distanceCM;
            return distanceCM;
        }

        if (!distanceFilterInitialised) {
            filteredDistanceCM = distanceCM;
            distanceFilterInitialised = true;
        } else {
            filteredDistanceCM =
                    (distanceAlpha * distanceCM)
                            + ((1.0 - distanceAlpha) * filteredDistanceCM);
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
        distanceFilterInitialised = false;
        filteredDistanceCM = 0.0;

        lastCommandedHood = clamp(RobotSettings.close, hoodMin, hoodMax);
        lastRawHoodTarget = lastCommandedHood;
        lastClippedHoodTarget = lastCommandedHood;

        hoodServo.setPosition(lastCommandedHood);
        blockServo.setPosition(RobotSettings.block);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}