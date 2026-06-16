package org.firstinspires.ftc.teamcode.opModes.subClasses;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {

    public Servo rotationalTurretServo = null;

    // ── Hold tuning ──────────────────────────────────────────────────────────
    public static double kP = 0.40;
    public static double maxStep = 0.16;
    public static double largeErrorThreshold = 0.10;
    public static boolean useLargeErrorFastMode = true;

    // ── Range / centre tuning ────────────────────────────────────────────────
    public static double turretScale = 1.15;
    public static double turretCentreOffset = 0.0;

    // Hard safety endpoints for servo travel
    public static double minServo = 0.00;
    public static double maxServo = 1.00;

    // ── Target pose in FIELD coordinates ─────────────────────────────────────
    public static Pose targetPose = new Pose(0, 0, 0);

    // ── Debug / telemetry values ─────────────────────────────────────────────
    public static double robotX = 0;
    public static double robotY = 0;
    public static double dx = 0;
    public static double dy = 0;
    public static double robotHeadingDeg = 0;
    public static double targetWorldAngle = 0;   // stored in DEGREES for telemetry
    public static double relativeAngleDeg = 0;
    public static double desiredServo = 0;
    public static double currentServo = 0;
    public static double servoError = 0;
    public static double appliedCorrection = 0;

    // ── Init ─────────────────────────────────────────────────────────────────
    public void init(@NonNull HardwareMap hardwareMap) {
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");
    }

    // ── Utility ──────────────────────────────────────────────────────────────
    public void centre() {
        double centre = clamp(0.5 + turretCentreOffset, minServo, maxServo);
        rotationalTurretServo.setPosition(centre);
    }

    public void setPose(Pose pose) {
        targetPose = pose;
    }

    // ── Main aim method ──────────────────────────────────────────────────────
    public void aimTurret(Pose robotPose) {
        robotX = robotPose.getX();
        robotY = robotPose.getY();

        double robotHeading = robotPose.getHeading();
        robotHeadingDeg = Math.toDegrees(robotHeading);

        // Field-space vector from robot to target
        dx = targetPose.getX() - robotX;
        dy = targetPose.getY() - robotY;

        // World angle to target
        double targetWorldAngleRad = Math.atan2(dy, dx);
        targetWorldAngle = Math.toDegrees(targetWorldAngleRad);

        // Robot-relative angle
        double relativeAngleRad = angleWrap(targetWorldAngleRad - robotHeading);
        relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        // Convert relative angle to servo target
        desiredServo = 0.5
                + turretCentreOffset
                + turretScale * (relativeAngleRad / Math.PI);

        desiredServo = clamp(desiredServo, minServo, maxServo);

        currentServo = rotationalTurretServo.getPosition();
        servoError = desiredServo - currentServo;

        double correction;
        if (useLargeErrorFastMode && Math.abs(servoError) > largeErrorThreshold) {
            correction = Math.signum(servoError) * maxStep;
        } else {
            correction = servoError * kP;
            correction = clamp(correction, -maxStep, maxStep);
        }

        appliedCorrection = correction;

        double newServoPos = currentServo + correction;
        newServoPos = clamp(newServoPos, minServo, maxServo);

        rotationalTurretServo.setPosition(newServoPos);
    }

    // ── Helpers ──────────────────────────────────────────────────────────────
    private double angleWrap(double angle) {
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
