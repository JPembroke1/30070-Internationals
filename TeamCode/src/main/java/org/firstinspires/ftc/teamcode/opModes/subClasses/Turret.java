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
    // Higher = stronger correction for small/medium error
    public static double kP = 0.40;

    // Max servo movement per loop
    // Higher = faster catch-up, but too high can overshoot
    public static double maxStep = 0.16;

    // If error exceeds this, turret uses full maxStep
    public static double largeErrorThreshold = 0.10;

    // Keep enabled for faster recovery when the bot turns quickly
    public static boolean useLargeErrorFastMode = true;

    // ── Range / centre tuning ────────────────────────────────────────────────
    // Use this to expand/reduce total rotation usage
    public static double turretScale = 1.15;

    // Use this to shift the whole turret centre left/right
    public static double turretCentreOffset = 0.0;

    // Hard safety endpoints for servo travel
    // Increase only if the mechanism is proven safe there
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
    public static double targetWorldAngleDeg = 0;
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
    /**
     * Field assumptions:
     * - +X = right
     * - +Y = up
     * - heading in radians
     *
     * targetPose and robotPose must both be in the same field coordinate system.
     */
    public void aimTurret(Pose robotPose) {
        robotX = robotPose.getX();
        robotY = robotPose.getY();

        double robotHeading = robotPose.getHeading();
        robotHeadingDeg = Math.toDegrees(robotHeading);

        // Field-space vector from robot to target
        dx = targetPose.getX() - robotX;
        dy = targetPose.getY() - robotY;

        // World angle to target
        double targetWorldAngle = Math.atan2(dy, dx);
        targetWorldAngleDeg = Math.toDegrees(targetWorldAngle);

        // Robot-relative angle
        double relativeAngle = angleWrap(targetWorldAngle - robotHeading);
        relativeAngleDeg = Math.toDegrees(relativeAngle);

        // Convert relative angle to servo target
        // '+' mapping is intentional because this was the working direction
        desiredServo = 0.5
                + turretCentreOffset
                + turretScale * (relativeAngle / Math.PI);

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