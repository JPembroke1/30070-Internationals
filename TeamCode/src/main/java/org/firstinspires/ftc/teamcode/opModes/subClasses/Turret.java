package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
public class Turret {

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────

    private Servo turretServo;

    public static String TURRET_SERVO_NAME = "turret";

    // ─────────────────────────────────────────────────────────────────────────
    // Servo tuning
    // ─────────────────────────────────────────────────────────────────────────

    public static double FORWARD_SERVO = 0.5;

    // ─────────────────────────────────────────────────────────────────────────
    // AIM OFFSET
    //
    // Applies a constant offset AFTER turret math.
    //
    // Tuning:
    // - too far RIGHT → decrease
    // - too far LEFT → increase
    //
    // Start with ±0.01 adjustments
    // ─────────────────────────────────────────────────────────────────────────

    public static double AIM_OFFSET = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Target
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(0, 0, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Telemetry values
    // ─────────────────────────────────────────────────────────────────────────

    public static double desiredServo = 0.5;
    public static double currentServo = 0.5;

    public static double servoError = 0.0;

    public static double leadX = 0.0;
    public static double leadY = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity compensation
    // ─────────────────────────────────────────────────────────────────────────

    private Supplier<Double> angularVelocitySupplier;

    private double robotVelocityX = 0.0;
    private double robotVelocityY = 0.0;

    public static boolean velocityCompensationActive = true;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap, Supplier<Double> angularVelocitySupplier) {
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        this.angularVelocitySupplier = angularVelocitySupplier;

        setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target control
    // ─────────────────────────────────────────────────────────────────────────

    public void setTargetPose(Pose pose) {
        targetPose = pose;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity input (field-relative)
    // ─────────────────────────────────────────────────────────────────────────

    public void setRobotVelocityField(double vx, double vy) {
        robotVelocityX = vx;
        robotVelocityY = vy;
    }

    public void clearRobotVelocity() {
        robotVelocityX = 0.0;
        robotVelocityY = 0.0;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Aim turret
    //
    // Calculates:
    // - angle to goal
    // - optional velocity lead
    // - converts to servo position
    // - applies AIM_OFFSET
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose, double vx, double vy) {

        if (robotPose == null || turretServo == null) {
            return;
        }

        double dx = targetPose.getX() - robotPose.getX();
        double dy = targetPose.getY() - robotPose.getY();

        // Velocity compensation
        if (velocityCompensationActive) {
            leadX = vx * 0.01;
            leadY = vy * 0.01;
        } else {
            leadX = 0.0;
            leadY = 0.0;
        }

        double compensatedX = dx + leadX;
        double compensatedY = dy + leadY;

        double angle = Math.atan2(compensatedY, compensatedX);

        // Convert angle to servo position
        double robotHeading = robotPose.getHeading();
        double relativeAngle = angle - robotHeading;

        // Normalise
        while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
        while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;

        desiredServo = FORWARD_SERVO + (relativeAngle / Math.PI) * 0.5;

        // ─────────────────────────────────────────────────────────
        // APPLY OFFSET HERE (FINAL STEP)
        // ─────────────────────────────────────────────────────────

        double commanded = desiredServo + AIM_OFFSET;

        commanded = clamp(commanded, 0.0, 1.0);

        turretServo.setPosition(commanded);

        currentServo = commanded;
        servoError = desiredServo - currentServo;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Forward position
    // ─────────────────────────────────────────────────────────────────────────

    public void setForward() {
        if (turretServo != null) {
            turretServo.setPosition(FORWARD_SERVO);
            currentServo = FORWARD_SERVO;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}