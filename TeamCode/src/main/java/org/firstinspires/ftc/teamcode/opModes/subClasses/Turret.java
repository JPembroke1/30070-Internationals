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
    //
    // FORWARD_SERVO:
    // - Position used when the turret should aim straight ahead.
    // - Start at 0.5.
    // - Tune this if the turret is not physically straight at 0.5.
    //
    // minServo / maxServo:
    // - Safety limits for servo travel.
    // ─────────────────────────────────────────────────────────────────────────

    public static double FORWARD_SERVO = 0.5;

    public static double minServo = 0.00;
    public static double maxServo = 1.00;

    // ─────────────────────────────────────────────────────────────────────────
    // Movement tuneables
    //
    // kP:
    // - Higher = moves more aggressively toward desired servo position.
    // - Lower = smoother but slower.
    //
    // minStep:
    // - Smallest allowed movement step near the target.
    //
    // maxStep:
    // - Largest allowed movement step when far from target.
    //
    // slowZone:
    // - Error range where movement begins slowing down.
    //
    // servoDeadband:
    // - If error is inside this range, turret will stop correcting.
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
    // - Controls how much servo position changes for a given angle.
    // - If turret under-aims, increase slightly.
    // - If turret over-aims, decrease slightly.
    // - If turret aims opposite, change this to negative.
    //
    // turretCentreOffset:
    // - Fine offset added to the forward/centre position.
    // ─────────────────────────────────────────────────────────────────────────

    public static double turretScale = 1.15;
    public static double turretCentreOffset = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity compensation tuneables
    //
    // USE_VELOCITY_COMPENSATION:
    // - true = aim slightly ahead/behind based on robot field velocity.
    // - false = aim directly at the target.
    //
    // SHOT_TIME_SECONDS:
    // - Estimated time from ball leaving shooter to reaching goal.
    // - Start around 0.20 to 0.35.
    // - Increase if moving shots still miss in direction of robot travel.
    // - Decrease if it over-compensates.
    //
    // VELOCITY_COMPENSATION_GAIN:
    // - 1.0 = full estimated compensation.
    // - 0.5 = half compensation.
    // - Start at 0.5 to be safe, then tune upward.
    //
    // MAX_LEAD_INCHES:
    // - Safety clamp so bad velocity data cannot create huge aim offsets.
    //
    // Important:
    // - robotVelocityX/Y must be FIELD velocity in inches per second.
    // - If you feed cm/sec or robot-centric velocity, compensation will be wrong.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_VELOCITY_COMPENSATION = true;

    public static double SHOT_TIME_SECONDS = 0.25;
    public static double VELOCITY_COMPENSATION_GAIN = 0.5;
    public static double MAX_LEAD_INCHES = 12.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Optional angular velocity compensation
    //
    // This is separate from translational velocity.
    // It predicts a small amount of robot heading change during ball flight.
    //
    // Leave disabled until translational compensation is working.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_ANGULAR_VELOCITY_COMPENSATION = false;
    public static double ANGULAR_COMPENSATION_GAIN = 0.5;

    private Supplier<Double> angularVelocitySupplier = null;

    // ─────────────────────────────────────────────────────────────────────────
    // Target pose
    //
    // TeleOp should own the target position.
    //
    // TeleOp should call:
    //      turret.setTargetPose(getTeleOpGoalPose());
    //
    // This default is only a fallback.
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(6, 138, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Robot velocity values
    //
    // These should be FIELD velocity values in inches per second.
    // ─────────────────────────────────────────────────────────────────────────

    public static double robotVelocityX = 0.0;
    public static double robotVelocityY = 0.0;
    public static double robotAngularVelocityRadPerSec = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Debug / telemetry values
    // ─────────────────────────────────────────────────────────────────────────

    public static double robotX = 0;
    public static double robotY = 0;

    public static double targetX = 6;
    public static double targetY = 138;

    public static double compensatedTargetX = 6;
    public static double compensatedTargetY = 138;

    public static double leadX = 0;
    public static double leadY = 0;
    public static double leadMagnitude = 0;

    public static double dx = 0;
    public static double dy = 0;

    public static double robotHeadingDeg = 0;
    public static double predictedHeadingDeg = 0;
    public static double targetWorldAngleDeg = 0;
    public static double relativeAngleDeg = 0;

    public static double desiredServo = 0.5;
    public static double currentServo = 0.5;
    public static double servoError = 0;
    public static double appliedCorrection = 0;
    public static double dynamicMaxStep = 0;

    public static boolean servoReady = false;
    public static boolean robotPoseValid = false;
    public static boolean targetPoseValid = false;
    public static boolean velocityDataActive = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    //
    // Standard init:
    // - Use this in TeleOp if you do not have angular velocity available.
    //
    // Overloaded init:
    // - Use this if your Auto/TeleOp already has follower::getAngularVelocity.
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        rotationalTurretServo = hardwareMap.get(Servo.class, "rotationalTurretServo");

        angularVelocitySupplier = null;

        targetX = targetPose.getX();
        targetY = targetPose.getY();
        compensatedTargetX = targetX;
        compensatedTargetY = targetY;
        targetPoseValid = true;

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
        servoError = 0;
        appliedCorrection = 0;
        dynamicMaxStep = 0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity input
    //
    // These values must be FIELD velocity in inches per second.
    //
    // Positive X/Y direction should match Pedro field coordinates.
    // ─────────────────────────────────────────────────────────────────────────

    public void setRobotVelocityField(double velocityXInchesPerSecond, double velocityYInchesPerSecond) {
        robotVelocityX = velocityXInchesPerSecond;
        robotVelocityY = velocityYInchesPerSecond;

        velocityDataActive = Math.abs(robotVelocityX) > 0.001 || Math.abs(robotVelocityY) > 0.001;
    }

    public void clearRobotVelocity() {
        robotVelocityX = 0.0;
        robotVelocityY = 0.0;
        robotAngularVelocityRadPerSec = 0.0;

        velocityDataActive = false;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Main aiming methods
    //
    // aimTurret(robotPose):
    // - Uses whatever velocity was last passed through setRobotVelocityField().
    //
    // aimTurret(robotPose, vx, vy):
    // - Convenience method.
    // - Updates velocity and then aims.
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
            appliedCorrection = 0;
            dynamicMaxStep = 0;
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
    // Simple model:
    // - If robot is moving in field +X, the ball inherits some +X velocity.
    // - To reduce that drift, aim slightly opposite the robot's movement.
    //
    // compensated target:
    //      target - robotVelocity * shotTime * gain
    //
    // This is intentionally tuneable and clamped.
    // ─────────────────────────────────────────────────────────────────────────

    private void calculateCompensatedTarget() {
        if (!USE_VELOCITY_COMPENSATION) {
            leadX = 0;
            leadY = 0;
            leadMagnitude = 0;

            compensatedTargetX = targetX;
            compensatedTargetY = targetY;
            return;
        }

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
    //
    // relativeAngleRad:
    // - 0 means target is straight ahead of the robot.
    //
    // Formula:
    // - FORWARD_SERVO is the centre/forward reference.
    // - turretCentreOffset fine-tunes mechanical centre.
    // - turretScale maps angle to servo movement.
    //
    // If turret aims backwards/opposite:
    // - Change turretScale from 1.15 to -1.15.
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