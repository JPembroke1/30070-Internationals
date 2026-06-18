package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
public class Turret {

    public Servo rotationalTurretServo = null;

    // ─────────────────────────────────────────────────────────────────────────
    // Servo position tuneables
    // ─────────────────────────────────────────────────────────────────────────

    public static double FORWARD_SERVO = 0.5;

    public static double minServo = 0.00;
    public static double maxServo = 1.00;

    // ─────────────────────────────────────────────────────────────────────────
    // Movement tuneables
    //
    // kP:
    // - Higher = more aggressive servo correction.
    // - Lower = smoother, slower correction.
    //
    // minStep:
    // - Smallest allowed move when close to target.
    //
    // maxStep:
    // - Largest allowed move when far from target.
    //
    // slowZone:
    // - Error range where movement slows down.
    //
    // servoDeadband:
    // - If error is smaller than this, turret stops correcting.
    // ─────────────────────────────────────────────────────────────────────────

    public static double kP = 1.8;

    public static double minStep = 0.004;
    public static double maxStep = 0.045;
    public static double slowZone = 0.08;
    public static double servoDeadband = 0.003;

    // ─────────────────────────────────────────────────────────────────────────
    // Angle-to-servo tuneables
    //
    // turretScale:
    // - If turret under-aims, increase slightly.
    // - If turret over-aims, decrease slightly.
    // - If turret aims opposite direction, make this negative.
    //
    // turretCentreOffset:
    // - Small mechanical centre correction.
    // ─────────────────────────────────────────────────────────────────────────

    public static double turretScale = 1.15;
    public static double turretCentreOffset = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity compensation tuneables
    //
    // USE_VELOCITY_COMPENSATION:
    // - Master switch.
    //
    // AUTO_DISABLE_VELOCITY_COMPENSATION:
    // - Turns compensation off when robot is nearly stationary.
    //
    // MIN_VELOCITY_FOR_COMPENSATION:
    // - Minimum field speed in inches/sec before compensation activates.
    //
    // SHOT_TIME_SECONDS:
    // - Estimated ball flight time.
    //
    // VELOCITY_COMPENSATION_GAIN:
    // - 1.0 = full compensation.
    // - 0.5 = half compensation.
    //
    // MAX_LEAD_INCHES:
    // - Safety clamp to prevent wild aim offsets.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_VELOCITY_COMPENSATION = true;
    public static boolean AUTO_DISABLE_VELOCITY_COMPENSATION = true;

    public static double MIN_VELOCITY_FOR_COMPENSATION = 2.0;

    public static double SHOT_TIME_SECONDS = 0.25;
    public static double VELOCITY_COMPENSATION_GAIN = 0.5;
    public static double MAX_LEAD_INCHES = 12.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Optional angular velocity compensation
    //
    // Leave disabled until normal velocity compensation is tuned.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_ANGULAR_VELOCITY_COMPENSATION = false;
    public static double ANGULAR_COMPENSATION_GAIN = 0.5;

    private Supplier<Double> angularVelocitySupplier = null;

    // ─────────────────────────────────────────────────────────────────────────
    // Target pose
    //
    // TeleOp / Auto should set this using:
    // turret.setTargetPose(goalPose);
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(6, 138, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Robot velocity values
    //
    // These must be field-relative inches/sec.
    // ─────────────────────────────────────────────────────────────────────────

    public static double robotVelocityX = 0.0;
    public static double robotVelocityY = 0.0;
    public static double robotSpeed = 0.0;

    public static double robotAngularVelocityRadPerSec = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Debug / telemetry values
    // ─────────────────────────────────────────────────────────────────────────

    public static double robotX = 0.0;
    public static double robotY = 0.0;

    public static double targetX = 6.0;
    public static double targetY = 138.0;

    public static double compensatedTargetX = 6.0;
    public static double compensatedTargetY = 138.0;

    public static double leadX = 0.0;
    public static double leadY = 0.0;
    public static double leadMagnitude = 0.0;

    public static double dx = 0.0;
    public static double dy = 0.0;

    public static double robotHeadingDeg = 0.0;
    public static double predictedHeadingDeg = 0.0;
    public static double targetWorldAngleDeg = 0.0;
    public static double relativeAngleDeg = 0.0;

    public static double desiredServo = 0.5;
    public static double currentServo = 0.5;
    public static double servoError = 0.0;
    public static double appliedCorrection = 0.0;
    public static double dynamicMaxStep = 0.0;

    public static boolean servoReady = false;
    public static boolean robotPoseValid = false;
    public static boolean targetPoseValid = false;

    public static boolean velocityDataActive = false;
    public static boolean velocityCompensationActive = false;
    public static boolean velocityCompensationAutoDisabled = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

        angularVelocitySupplier = null;

        targetX = targetPose.getX();
        targetY = targetPose.getY();

        compensatedTargetX = targetX;
        compensatedTargetY = targetY;

        targetPoseValid = true;

        clearRobotVelocity();
        setForward();
    }

    public void init(HardwareMap hardwareMap, Supplier<Double> angularVelocitySupplier) {
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

        this.angularVelocitySupplier = angularVelocitySupplier;

        targetX = targetPose.getX();
        targetY = targetPose.getY();

        compensatedTargetX = targetX;
        compensatedTargetY = targetY;

        targetPoseValid = true;

        clearRobotVelocity();
        setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Basic controls
    // ─────────────────────────────────────────────────────────────────────────

    public void centre() {
        setForward();
    }

    public void setForward() {
        setServoDirect(FORWARD_SERVO);
    }

    public void setPose(Pose pose) {
        setTargetPose(pose);
    }

    public void setTargetPose(Pose pose) {
        if (pose == null) {
            targetPoseValid = false;
            return;
        }

        targetPose = pose;

        targetX = pose.getX();
        targetY = pose.getY();

        compensatedTargetX = targetX;
        compensatedTargetY = targetY;

        targetPoseValid = true;
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    public void setServoDirect(double position) {
        double clipped = clamp(position, minServo, maxServo);

        if (rotationalTurretServo != null) {
            rotationalTurretServo.setPosition(clipped);
            servoReady = true;
        } else {
            servoReady = false;
        }

        currentServo = clipped;
        desiredServo = clipped;
        servoError = 0.0;
        appliedCorrection = 0.0;
        dynamicMaxStep = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity input
    //
    // These values should be field-relative inches/sec.
    // ─────────────────────────────────────────────────────────────────────────

    public void setRobotVelocityField(double velocityXInchesPerSecond, double velocityYInchesPerSecond) {
        robotVelocityX = velocityXInchesPerSecond;
        robotVelocityY = velocityYInchesPerSecond;

        robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);

        velocityDataActive = robotSpeed > 0.001;
    }

    public void clearRobotVelocity() {
        robotVelocityX = 0.0;
        robotVelocityY = 0.0;
        robotSpeed = 0.0;

        robotAngularVelocityRadPerSec = 0.0;

        velocityDataActive = false;
        velocityCompensationActive = false;
        velocityCompensationAutoDisabled = false;

        leadX = 0.0;
        leadY = 0.0;
        leadMagnitude = 0.0;

        compensatedTargetX = targetX;
        compensatedTargetY = targetY;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main aiming methods
    //
    // aimTurret(robotPose):
    // - Uses last velocity values passed by setRobotVelocityField().
    //
    // aimTurret(robotPose, vx, vy):
    // - Updates velocity, then aims.
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose) {
        aimTurretInternal(robotPose);
    }

    public void aimTurret(Pose robotPose, double velocityXInchesPerSecond, double velocityYInchesPerSecond) {
        setRobotVelocityField(velocityXInchesPerSecond, velocityYInchesPerSecond);
        aimTurretInternal(robotPose);
    }

    private void aimTurretInternal(Pose robotPose) {
        if (rotationalTurretServo == null) {
            servoReady = false;
            return;
        }

        servoReady = true;

        if (robotPose == null) {
            robotPoseValid = false;
            return;
        }

        robotPoseValid = true;

        if (targetPose == null) {
            targetPoseValid = false;
            return;
        }

        targetPoseValid = true;

        robotX = robotPose.getX();
        robotY = robotPose.getY();

        targetX = targetPose.getX();
        targetY = targetPose.getY();

        double robotHeadingRad = robotPose.getHeading();
        robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        updateAngularVelocity();

        double predictedHeadingRad = robotHeadingRad;

        if (USE_ANGULAR_VELOCITY_COMPENSATION) {
            predictedHeadingRad += robotAngularVelocityRadPerSec
                    * SHOT_TIME_SECONDS
                    * ANGULAR_COMPENSATION_GAIN;
        }

        predictedHeadingDeg = Math.toDegrees(predictedHeadingRad);

        calculateCompensatedTarget();

        dx = compensatedTargetX - robotX;
        dy = compensatedTargetY - robotY;

        double targetWorldAngleRad = Math.atan2(dy, dx);
        targetWorldAngleDeg = Math.toDegrees(targetWorldAngleRad);

        double relativeAngleRad = angleWrap(targetWorldAngleRad - predictedHeadingRad);
        relativeAngleDeg = Math.toDegrees(relativeAngleRad);

        desiredServo = angleToServo(relativeAngleRad);
        currentServo = rotationalTurretServo.getPosition();
        servoError = desiredServo - currentServo;

        double absError = Math.abs(servoError);

        if (absError <= servoDeadband) {
            appliedCorrection = 0.0;
            dynamicMaxStep = 0.0;
            return;
        }

        dynamicMaxStep = calculateDynamicMaxStep(absError);

        double correction = servoError * kP;
        correction = clamp(correction, -dynamicMaxStep, dynamicMaxStep);

        appliedCorrection = correction;

        double newServoPosition = currentServo + correction;
        newServoPosition = clamp(newServoPosition, minServo, maxServo);

        rotationalTurretServo.setPosition(newServoPosition);
        currentServo = newServoPosition;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity compensation
    //
    // lead = velocity × shot time × gain
    //
    // compensated target = target - lead
    //
    // If the robot is moving right, the ball carries rightward velocity.
    // So we aim slightly left by subtracting the velocity lead.
    // ─────────────────────────────────────────────────────────────────────────

    private void calculateCompensatedTarget() {
        if (!USE_VELOCITY_COMPENSATION) {
            disableVelocityCompensation(false);
            return;
        }

        robotSpeed = Math.hypot(robotVelocityX, robotVelocityY);

        if (AUTO_DISABLE_VELOCITY_COMPENSATION
                && robotSpeed < MIN_VELOCITY_FOR_COMPENSATION) {
            disableVelocityCompensation(true);
            return;
        }

        velocityCompensationActive = true;
        velocityCompensationAutoDisabled = false;

        leadX = robotVelocityX * SHOT_TIME_SECONDS * VELOCITY_COMPENSATION_GAIN;
        leadY = robotVelocityY * SHOT_TIME_SECONDS * VELOCITY_COMPENSATION_GAIN;

        leadMagnitude = Math.hypot(leadX, leadY);

        if (leadMagnitude > MAX_LEAD_INCHES && leadMagnitude > 0.0) {
            double scale = MAX_LEAD_INCHES / leadMagnitude;

            leadX *= scale;
            leadY *= scale;

            leadMagnitude = MAX_LEAD_INCHES;
        }

        compensatedTargetX = targetX - leadX;
        compensatedTargetY = targetY - leadY;
    }

    private void disableVelocityCompensation(boolean autoDisabled) {
        velocityCompensationActive = false;
        velocityCompensationAutoDisabled = autoDisabled;

        leadX = 0.0;
        leadY = 0.0;
        leadMagnitude = 0.0;

        compensatedTargetX = targetX;
        compensatedTargetY = targetY;
    }

    private void updateAngularVelocity() {
        if (angularVelocitySupplier == null) {
            robotAngularVelocityRadPerSec = 0.0;
            return;
        }

        Double suppliedAngularVelocity = angularVelocitySupplier.get();

        if (suppliedAngularVelocity == null) {
            robotAngularVelocityRadPerSec = 0.0;
            return;
        }

        robotAngularVelocityRadPerSec = suppliedAngularVelocity;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Conversion helpers
    // ─────────────────────────────────────────────────────────────────────────

    private double angleToServo(double relativeAngleRad) {
        double servoPosition = FORWARD_SERVO
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