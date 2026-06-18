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

    public static String TURRET_SERVO_NAME = "turret";

    // ─────────────────────────────────────────────────────────────────────────
    // Mechanical Servo Calibration
    //
    // These are the real safe servo limits for your physical turret.
    //
    // LEFT_SERVO_LIMIT:
    // - servo position when turret is safely at -90°
    //
    // RIGHT_SERVO_LIMIT:
    // - servo position when turret is safely at +90°
    //
    // Do not assume 0.0 and 1.0 are always safe on the real robot.
    // Start conservative if the turret can hit the frame.
    // ─────────────────────────────────────────────────────────────────────────

    public static double LEFT_SERVO_LIMIT = 0.0;
    public static double RIGHT_SERVO_LIMIT = 1.0;

    // If the turret moves the wrong way, change this to true.
    public static boolean SERVO_REVERSED = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Tuning
    //
    // AIM_OFFSET:
    // - final trim added after all turret maths
    // - adjusted live in TeleOp using gamepad2 dpad left/right
    //
    // AIM_GAIN:
    // - scales the robot-relative angle before converting to servo position
    //
    // Recommended:
    // - leave AIM_GAIN at 1.0 unless the turret consistently under-rotates
    //   or over-rotates across the whole range.
    //
    // If turret under-aims:
    // - increase AIM_GAIN slightly, for example 1.02
    //
    // If turret over-aims:
    // - decrease AIM_GAIN slightly, for example 0.98
    //
    // Confidence:
    // - AIM_OFFSET is high-confidence for small final shot trim.
    // - AIM_GAIN should be used carefully because it changes the whole angle map.
    // ─────────────────────────────────────────────────────────────────────────

    public static double AIM_OFFSET = 0.015;
    public static double AIM_GAIN = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret Angle Limits
    //
    // This locks the maths to a 180° turret:
    //
    // -90° = left limit
    //   0° = forward
    // +90° = right limit
    // ─────────────────────────────────────────────────────────────────────────

    private static final double MIN_TURRET_ANGLE = -Math.PI / 2.0;
    private static final double MAX_TURRET_ANGLE =  Math.PI / 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Target
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(0, 0, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity Compensation
    //
    // velocityCompensationActive:
    // - true = aim slightly ahead based on robot field velocity
    // - false = aim directly at target
    //
    // VELOCITY_LEAD_GAIN:
    // - how strongly robot velocity affects turret aim
    //
    // Start low.
    // If shots miss behind while moving, increase slightly.
    // If shots miss ahead while moving, decrease slightly.
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean velocityCompensationActive = true;
    public static double VELOCITY_LEAD_GAIN = 0.01;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
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

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Turret
    //
    // Process:
    // 1. Calculate field angle from robot to goal
    // 2. Add simple velocity lead if enabled
    // 3. Convert field angle to robot-relative angle
    // 4. Normalise to -π to +π
    // 5. Apply AIM_GAIN
    // 6. Clamp to -90° to +90°
    // 7. Convert angle to 0.0 to 1.0 normalised position
    // 8. Apply servo reversal if needed
    // 9. Map to calibrated servo limits
    // 10. Apply AIM_OFFSET
    // 11. Clamp to calibrated servo limits
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose, double vx, double vy) {

        if (robotPose == null || turretServo == null) {
            return;
        }

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        double leadX = velocityCompensationActive ? vx * VELOCITY_LEAD_GAIN : 0.0;
        double leadY = velocityCompensationActive ? vy * VELOCITY_LEAD_GAIN : 0.0;

        double targetAngle = Math.atan2(dy + leadY, dx + leadX);

        double relativeAngle = targetAngle - robotPose.getHeading();

        relativeAngle = normaliseRadians(relativeAngle);

        relativeAngle *= AIM_GAIN;

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
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Forward Position
    //
    // Forward is the centre between the calibrated servo limits.
    // This should point the turret straight ahead if the horn/linkage is centred.
    // ─────────────────────────────────────────────────────────────────────────

    public void setForward() {
        if (turretServo == null) {
            return;
        }

        turretServo.setPosition(getCalibratedCentrePosition());
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