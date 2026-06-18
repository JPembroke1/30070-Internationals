package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
public class Turret {

    public Servo rotationalTurretServo = null;

    // ─────────────────────────────────────────────────────────
    // Servo tuning
    // ─────────────────────────────────────────────────────────

    public static double FORWARD_SERVO = 0.5;
    public static double minServo = 0.0;
    public static double maxServo = 1.0;

    // ─────────────────────────────────────────────────────────
    // Movement tuning
    // ─────────────────────────────────────────────────────────

    public static double kP = 1.8;
    public static double minStep = 0.004;
    public static double maxStep = 0.045;
    public static double slowZone = 0.08;
    public static double servoDeadband = 0.003;

    // ─────────────────────────────────────────────────────────
    // Angle tuning
    // ─────────────────────────────────────────────────────────

    public static double turretScale = 1.15;
    public static double turretCentreOffset = 0.0;

    // ─────────────────────────────────────────────────────────
    // Velocity compensation
    // ─────────────────────────────────────────────────────────

    public static boolean USE_VELOCITY_COMP = true;
    public static boolean AUTO_DISABLE = true;

    public static double MIN_SPEED = 2.0; // in/s

    public static double SHOT_TIME = 0.25;         // seconds
    public static double VELOCITY_GAIN = 0.5;      // start conservative
    public static double MAX_LEAD = 12.0;          // inches

    // ─────────────────────────────────────────────────────────
    // Angular compensation
    // ─────────────────────────────────────────────────────────

    public static boolean USE_ANGULAR_COMP = true;   // enabled
    public static double ANGULAR_GAIN = 0.4;         // small

    private Supplier<Double> angularVelocitySupplier;

    // ─────────────────────────────────────────────────────────
    // Target
    // ─────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(6, 138, 0);

    // ─────────────────────────────────────────────────────────
    // Velocity data
    // ─────────────────────────────────────────────────────────

    public static double vx = 0;
    public static double vy = 0;
    public static double speed = 0;

    public static double angularVelocity = 0;

    // ─────────────────────────────────────────────────────────
    // Debug
    // ─────────────────────────────────────────────────────────

    public static double leadX = 0;
    public static double leadY = 0;
    public static boolean compActive = false;

    public static double desiredServo = 0.5;
    public static double currentServo = 0.5;
    public static double servoError = 0;

    // ─────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────

    public void init(HardwareMap hw) {
        rotationalTurretServo = hw.get(Servo.class, "rotationalTurretServo");
        setForward();
    }

    public void init(HardwareMap hw, Supplier<Double> angVel) {
        rotationalTurretServo = hw.get(Servo.class, "rotationalTurretServo");
        angularVelocitySupplier = angVel;
        setForward();
    }

    // ─────────────────────────────────────────────────────────
    // Basic
    // ─────────────────────────────────────────────────────────

    public void setForward() {
        setServoDirect(FORWARD_SERVO);
    }

    public void setTargetPose(Pose pose) {
        if (pose != null) targetPose = pose;
    }

    public void setServoDirect(double pos) {
        double clipped = clamp(pos, minServo, maxServo);
        rotationalTurretServo.setPosition(clipped);
        currentServo = clipped;
    }

    public void setRobotVelocityField(double x, double y) {
        vx = x;
        vy = y;
        speed = Math.hypot(vx, vy);
    }

    // ─────────────────────────────────────────────────────────
    // Main aiming
    // ─────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose, double velX, double velY) {

        setRobotVelocityField(velX, velY);

        if (angularVelocitySupplier != null) {
            Double val = angularVelocitySupplier.get();
            angularVelocity = (val != null) ? val : 0;
        }

        double robotHeading = robotPose.getHeading();

        // ─── Velocity auto-disable ───
        if (AUTO_DISABLE && speed < MIN_SPEED) {
            compActive = false;
            leadX = 0;
            leadY = 0;
        } else if (USE_VELOCITY_COMP) {
            compActive = true;

            leadX = vx * SHOT_TIME * VELOCITY_GAIN;
            leadY = vy * SHOT_TIME * VELOCITY_GAIN;

            double mag = Math.hypot(leadX, leadY);
            if (mag > MAX_LEAD && mag > 0) {
                double scale = MAX_LEAD / mag;
                leadX *= scale;
                leadY *= scale;
            }
        } else {
            compActive = false;
            leadX = 0;
            leadY = 0;
        }

        // ─── Adjust target ───
        double tx = targetPose.getX() - leadX;
        double ty = targetPose.getY() - leadY;

        // ─── Predict heading change (angular comp) ───
        double predictedHeading = robotHeading;

        if (USE_ANGULAR_COMP) {
            predictedHeading += angularVelocity * SHOT_TIME * ANGULAR_GAIN;
        }

        // ─── Angle calculation ───
        double dx = tx - robotPose.getX();
        double dy = ty - robotPose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double relativeAngle = wrap(targetAngle - predictedHeading);

        double targetServo = angleToServo(relativeAngle);

        currentServo = rotationalTurretServo.getPosition();
        servoError = targetServo - currentServo;

        if (Math.abs(servoError) <= servoDeadband) return;

        double step = dynamicStep(Math.abs(servoError));

        double correction = clamp(servoError * kP, -step, step);

        double newPos = clamp(currentServo + correction, minServo, maxServo);

        rotationalTurretServo.setPosition(newPos);
        currentServo = newPos;
    }

    // ─────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────

    private double angleToServo(double angle) {
        return clamp(
                FORWARD_SERVO + turretCentreOffset + turretScale * (angle / Math.PI),
                minServo, maxServo
        );
    }

    private double dynamicStep(double err) {
        if (err >= slowZone) return maxStep;
        return minStep + (maxStep - minStep) * (err / slowZone);
    }

    private double wrap(double a) {
        while (a <= -Math.PI) a += 2 * Math.PI;
        while (a > Math.PI) a -= 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}