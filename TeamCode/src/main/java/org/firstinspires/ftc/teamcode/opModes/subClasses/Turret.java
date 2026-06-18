package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;

import java.util.function.DoubleSupplier;

@Configurable
public class Turret {

    public Servo rotationalTurretServo = null;

    // ─────────────────────────────────────────────────────────────────────────
    // Movement tuning
    // Tuning notes:
    // - Increase maxStep for faster large movements.
    // - Increase servoDeadband if turret jitters near target.
    // - Decrease slowZone if turret slows down too early.
    // - Decrease kP if it overshoots or snaps too aggressively.
    // ─────────────────────────────────────────────────────────────────────────

    public static double kP = 1.8;

    public static double minStep = 0.004;
    public static double maxStep = 0.045;
    public static double slowZone = 0.08;
    public static double servoDeadband = 0.003;

    // ─────────────────────────────────────────────────────────────────────────
    // Range / centre tuning
    // Centre is 0.5, so turretCentreOffset should stay 0.0 unless physically needed.
    // Tune turretScale if centre is correct but side angles under/over aim.
    // ─────────────────────────────────────────────────────────────────────────

    public static double turretScale = 1.15;
    public static double turretCentreOffset = 0.0;

    public static double minServo = 0.00;
    public static double maxServo = 1.00;

    // ─────────────────────────────────────────────────────────────────────────
    // Target pose in field coordinates
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(6, 138, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Debug / telemetry values
    // ─────────────────────────────────────────────────────────────────────────

    public static double robotX = 0;
    public static double robotY = 0;
    public static double dx = 0;
    public static double dy = 0;

    public static double robotHeadingDeg = 0;
    public static double targetWorldAngle = 0;
    public static double relativeAngleDeg = 0;

    public static double desiredServo = 0;
    public static double currentServo = 0;
    public static double servoError = 0;
    public static double appliedCorrection = 0;
    public static double dynamicMaxStep = 0;

    private DoubleSupplier angularVelocity;

    public static double angularVelocityCorrection = 0.2;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap, DoubleSupplier angularVelocity) {
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");
        this.angularVelocity = angularVelocity;
        centre();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Basic controls
    // ─────────────────────────────────────────────────────────────────────────

    public void centre() {
        double centre = clamp(0.5 + turretCentreOffset, minServo, maxServo);

        if (rotationalTurretServo != null) {
            rotationalTurretServo.setPosition(centre);
        }

        currentServo = centre;
        desiredServo = centre;
        servoError = 0;
        appliedCorrection = 0;
        dynamicMaxStep = 0;
    }

    public void setPose(Pose pose) {
        targetPose = pose;
    }

    public void setTargetPose(Pose pose) {
        targetPose = pose;
    }

    public void setServoDirect(double position) {
        double clipped = clamp(position, minServo, maxServo);

        if (rotationalTurretServo != null) {
            rotationalTurretServo.setPosition(clipped);
        }

        currentServo = clipped;
        desiredServo = clipped;
        servoError = 0;
        appliedCorrection = 0;
        dynamicMaxStep = 0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main aiming method
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose) {
        if (rotationalTurretServo == null || robotPose == null || targetPose == null) {
            return;
        }

        robotX = robotPose.getX();
        robotY = robotPose.getY();

        double robotHeading = robotPose.getHeading();
        robotHeadingDeg = Math.toDegrees(robotHeading);

        dx = targetPose.getX() - robotX;
        dy = targetPose.getY() - robotY;

        double targetWorldAngleRad = Math.atan2(dy, dx) + angularVelocity.getAsDouble() * angularVelocityCorrection;
        targetWorldAngle = Math.toDegrees(targetWorldAngleRad);

        double relativeAngleRad = angleWrap(targetWorldAngleRad - robotHeading);
        relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        desiredServo = angleToServo(relativeAngleRad);
        currentServo = rotationalTurretServo.getPosition();
        servoError = desiredServo - currentServo;

        double absError = Math.abs(servoError);

        if (absError <= servoDeadband) {
            appliedCorrection = 0;
            dynamicMaxStep = 0;
            return;
        }

        dynamicMaxStep = calculateDynamicMaxStep(absError);

        double correction = servoError * kP;
        correction = clamp(correction, -dynamicMaxStep, dynamicMaxStep);

        appliedCorrection = correction;

        double newServoPos = currentServo + correction;
        newServoPos = clamp(newServoPos, minServo, maxServo);

        rotationalTurretServo.setPosition(newServoPos);
        currentServo = newServoPos;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Conversion helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double angleToServo(double relativeAngleRad) {
        double servoPosition = 0.5
                + turretCentreOffset
                + turretScale * (relativeAngleRad / Math.PI);

        return clamp(servoPosition, minServo, maxServo);
    }

    private double calculateDynamicMaxStep(double absError) {
        if (absError >= slowZone) {
            return maxStep;
        }

        double scale = absError / slowZone;
        return minStep + ((maxStep - minStep) * scale);
    }

    public boolean isAtTarget() {
        return Math.abs(servoError) <= servoDeadband;
    }

    public boolean isAtTarget(double tolerance) {
        return Math.abs(servoError) <= tolerance;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Math helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double angleWrap(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}