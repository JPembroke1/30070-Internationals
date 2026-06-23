package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────

    private Servo turretServo;
    private LimelightAssist limelightAssist;

    public static String TURRET_SERVO_NAME = "turret";

    // ─────────────────────────────────────────────────────────────────────────
    // Mechanical Servo Calibration
    // ─────────────────────────────────────────────────────────────────────────

    public static double LEFT_SERVO_LIMIT = 0.0;
    public static double RIGHT_SERVO_LIMIT = 1.0;

    public static boolean SERVO_REVERSED = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Tuning
    //
    // AIM_OFFSET is still a servo-position trim.
    // Do not treat AIM_OFFSET as degrees or radians.
    // ─────────────────────────────────────────────────────────────────────────

    public static double AIM_OFFSET = 0.0;
    public static double AIM_GAIN = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret Angle Limits
    // ─────────────────────────────────────────────────────────────────────────

    private static final double MIN_TURRET_ANGLE = -Math.PI / 2.0;
    private static final double MAX_TURRET_ANGLE = Math.PI / 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Target
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(0, 0, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity Compensation
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean velocityCompensationActive = true;
    public static double VELOCITY_LEAD_GAIN = 0.01;

    // ─────────────────────────────────────────────────────────────────────────
    // Limelight Assist
    //
    // Default is OFF.
    //
    // Enable both:
    // Turret.USE_LIMELIGHT_ASSIST = true;
    // LimelightAssist.LIMELIGHT_ENABLED = true;
    //
    // If correction moves the wrong way:
    // LimelightAssist.CORRECTION_DIRECTION = -1.0;
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_LIMELIGHT_ASSIST = false;

    public static boolean UPDATE_LIMELIGHT_INSIDE_TURRET = true;
    public static boolean REQUIRE_LIMELIGHT_TARGET = true;
    public static boolean DISABLE_LIMELIGHT_WHEN_LOCKED = false;

    public static double LIMELIGHT_CORRECTION_GAIN = 1.0;
    public static double MAX_LIMELIGHT_CORRECTION_RAD = Math.toRadians(5.0);

    public static double VISION_SMOOTHING_ALPHA = 0.35;
    public static double MAX_VISION_STEP_RAD = Math.toRadians(1.5);

    public static boolean limelightAssistActive = false;
    public static boolean limelightTargetVisible = false;
    public static boolean limelightLocked = false;

    public static double lastLimelightTx = 0.0;
    public static double lastRawLimelightCorrectionDeg = 0.0;
    public static double lastAppliedLimelightCorrectionDeg = 0.0;

    private double previousVisionCorrectionRad = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Diagnostics
    // ─────────────────────────────────────────────────────────────────────────

    public static double lastRobotX = 0.0;
    public static double lastRobotY = 0.0;
    public static double lastRobotHeadingDeg = 0.0;

    public static double lastTargetX = 0.0;
    public static double lastTargetY = 0.0;

    public static double lastFieldTargetAngleDeg = 0.0;
    public static double lastRelativeAngleDeg = 0.0;
    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;
    public static double lastFinalAngleDeg = 0.0;
    public static double lastServoPosition = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        limelightAssist = new LimelightAssist();
        limelightAssist.init(hardwareMap);
        limelightAssist.setEnabled(USE_LIMELIGHT_ASSIST && LimelightAssist.LIMELIGHT_ENABLED);

        setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target Control
    // ─────────────────────────────────────────────────────────────────────────

    public void setTargetPose(Pose pose) {
        if (pose != null) {
            targetPose = pose;
        }
    }

    public LimelightAssist getLimelightAssist() {
        return limelightAssist;
    }

    public void setLimelightAssistEnabled(boolean enabled) {
        USE_LIMELIGHT_ASSIST = enabled;

        if (limelightAssist != null) {
            limelightAssist.setEnabled(enabled);
        }

        if (!enabled) {
            resetVisionCorrection();
        }
    }

    public void toggleLimelightAssist() {
        setLimelightAssistEnabled(!USE_LIMELIGHT_ASSIST);
    }

    public void updateLimelightOnly() {
        if (limelightAssist != null) {
            limelightAssist.update();
        }
    }

    public void stopLimelight() {
        if (limelightAssist != null) {
            limelightAssist.stop();
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Turret
    //
    // Process:
    // 1. Calculate field angle from robot to goal
    // 2. Add simple velocity lead if enabled
    // 3. Convert field angle to robot-relative angle
    // 4. Normalise to -π to +π
    // 5. Apply AIM_GAIN
    // 6. Add optional Limelight angular correction
    // 7. Clamp to -90° to +90°
    // 8. Convert angle to 0.0 to 1.0 normalised position
    // 9. Apply servo reversal if needed
    // 10. Map to calibrated servo limits
    // 11. Apply AIM_OFFSET as servo-position trim
    // 12. Clamp to calibrated servo limits
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose, double vx, double vy) {

        if (robotPose == null || turretServo == null) {
            return;
        }

        if (UPDATE_LIMELIGHT_INSIDE_TURRET && limelightAssist != null) {
            limelightAssist.update();
        }

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double leadX = velocityCompensationActive ? vx * VELOCITY_LEAD_GAIN : 0.0;
        double leadY = velocityCompensationActive ? vy * VELOCITY_LEAD_GAIN : 0.0;

        double targetAngle = Math.atan2(dy + leadY, dx + leadX);

        double relativeAngle = targetAngle - robotPose.getHeading();

        relativeAngle = normaliseRadians(relativeAngle);

        relativeAngle *= AIM_GAIN;

        double limelightCorrectionRad = calculateLimelightCorrectionRad();

        relativeAngle += limelightCorrectionRad;

        relativeAngle = clamp(
                relativeAngle,
                MIN_TURRET_ANGLE,
                MAX_TURRET_ANGLE
        );

        double normalisedPosition = angleToNormalisedPosition(relativeAngle);

        if (SERVO_REVERSED) {
            normalisedPosition = 1.0 - normalisedPosition;
        }

        double servoPosition = mapNormalisedToServoLimits(normalisedPosition);

        servoPosition += AIM_OFFSET;

        servoPosition = clampToServoLimits(servoPosition);

        turretServo.setPosition(servoPosition);

        lastRobotX = robotPose.getX();
        lastRobotY = robotPose.getY();
        lastRobotHeadingDeg = Math.toDegrees(robotPose.getHeading());

        lastTargetX = targetPose.getX();
        lastTargetY = targetPose.getY();

        lastVelocityLeadX = leadX;
        lastVelocityLeadY = leadY;

        lastFieldTargetAngleDeg = Math.toDegrees(targetAngle);
        lastRelativeAngleDeg = Math.toDegrees(relativeAngle);
        lastFinalAngleDeg = Math.toDegrees(relativeAngle);
        lastServoPosition = servoPosition;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Limelight Correction
    // ─────────────────────────────────────────────────────────────────────────

    private double calculateLimelightCorrectionRad() {
        limelightTargetVisible = false;
        limelightLocked = false;

        if (!USE_LIMELIGHT_ASSIST || limelightAssist == null) {
            limelightAssistActive = false;
            resetVisionCorrection();
            return 0.0;
        }

        limelightTargetVisible = limelightAssist.hasTarget();
        limelightLocked = limelightAssist.isVisionLocked();
        lastLimelightTx = limelightAssist.getTx();

        if (REQUIRE_LIMELIGHT_TARGET && !limelightTargetVisible) {
            limelightAssistActive = false;
            resetVisionCorrection();
            return 0.0;
        }

        if (DISABLE_LIMELIGHT_WHEN_LOCKED && limelightLocked) {
            limelightAssistActive = false;
            resetVisionCorrection();
            return 0.0;
        }

        double rawCorrectionRad =
                limelightAssist.getTurretCorrectionRadians()
                        * LIMELIGHT_CORRECTION_GAIN;

        rawCorrectionRad = clamp(
                rawCorrectionRad,
                -MAX_LIMELIGHT_CORRECTION_RAD,
                MAX_LIMELIGHT_CORRECTION_RAD
        );

        lastRawLimelightCorrectionDeg = Math.toDegrees(rawCorrectionRad);

        double alpha = clamp(VISION_SMOOTHING_ALPHA, 0.0, 1.0);

        double smoothedCorrectionRad =
                (alpha * rawCorrectionRad)
                        + ((1.0 - alpha) * previousVisionCorrectionRad);

        double stepRad = smoothedCorrectionRad - previousVisionCorrectionRad;

        stepRad = clamp(
                stepRad,
                -MAX_VISION_STEP_RAD,
                MAX_VISION_STEP_RAD
        );

        double appliedCorrectionRad = previousVisionCorrectionRad + stepRad;

        appliedCorrectionRad = clamp(
                appliedCorrectionRad,
                -MAX_LIMELIGHT_CORRECTION_RAD,
                MAX_LIMELIGHT_CORRECTION_RAD
        );

        previousVisionCorrectionRad = appliedCorrectionRad;

        lastAppliedLimelightCorrectionDeg = Math.toDegrees(appliedCorrectionRad);
        limelightAssistActive = true;

        return appliedCorrectionRad;
    }

    private void resetVisionCorrection() {
        previousVisionCorrectionRad = 0.0;

        limelightAssistActive = false;
        limelightTargetVisible = false;
        limelightLocked = false;

        lastLimelightTx = 0.0;
        lastRawLimelightCorrectionDeg = 0.0;
        lastAppliedLimelightCorrectionDeg = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Forward Position
    // ─────────────────────────────────────────────────────────────────────────

    public void setForward() {
        if (turretServo == null) {
            return;
        }

        resetVisionCorrection();
        turretServo.setPosition(getCalibratedCentrePosition());
        lastServoPosition = getCalibratedCentrePosition();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Calibration Helpers
    // ─────────────────────────────────────────────────────────────────────────

    public static double getCalibratedCentrePosition() {
        return (LEFT_SERVO_LIMIT + RIGHT_SERVO_LIMIT) / 2.0;
    }

    private double angleToNormalisedPosition(double angleRadians) {
        return (angleRadians - MIN_TURRET_ANGLE) /
                (MAX_TURRET_ANGLE - MIN_TURRET_ANGLE);
    }

    private double mapNormalisedToServoLimits(double normalisedPosition) {
        double minServo = Math.min(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);
        double maxServo = Math.max(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        return minServo + normalisedPosition * (maxServo - minServo);
    }

    private double clampToServoLimits(double value) {
        double minServo = Math.min(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);
        double maxServo = Math.max(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        return clamp(value, minServo, maxServo);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Angle Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double normaliseRadians(double angleRadians) {
        while (angleRadians > Math.PI) {
            angleRadians -= 2.0 * Math.PI;
        }

        while (angleRadians < -Math.PI) {
            angleRadians += 2.0 * Math.PI;
        }

        return angleRadians;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // General Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}