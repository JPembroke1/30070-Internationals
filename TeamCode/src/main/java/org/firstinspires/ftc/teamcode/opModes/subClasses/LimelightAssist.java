package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class LimelightAssist {

    private Limelight3A limelight = null;

    public static String LIMELIGHT_NAME = "limelight";

    public static boolean LIMELIGHT_ENABLED = false;

    public static int PIPELINE_INDEX = 0;
    public static int POLL_RATE_HZ = 100;

    public static double CAMERA_TO_SHOOTER_OFFSET_DEG = 0.0;

    public static double TX_DEADBAND_DEG = 0.25;
    public static double VISION_LOCK_TOLERANCE_DEG = 1.0;

    public static double CORRECTION_GAIN = 1.0;
    public static double CORRECTION_DIRECTION = 1.0;

    public static double MAX_CORRECTION_DEG = 5.0;

    public static double MIN_TARGET_AREA = 0.0;

    public static boolean connected = false;
    public static boolean running = false;
    public static boolean validTarget = false;
    public static boolean visionLocked = false;

    public static double tx = 0.0;
    public static double ty = 0.0;
    public static double ta = 0.0;

    public static double rawCorrectionDeg = 0.0;
    public static double correctionDeg = 0.0;

    public static double lastValidTx = 0.0;
    public static double lastValidTy = 0.0;
    public static double lastValidTa = 0.0;

    public static double lastUpdateSeconds = 0.0;

    public void init(HardwareMap hardwareMap) {
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);

            limelight.setPollRateHz(POLL_RATE_HZ);
            limelight.pipelineSwitch(PIPELINE_INDEX);

            if (LIMELIGHT_ENABLED) {
                limelight.start();
            } else {
                limelight.pause();
            }

            updateConnectionState();
        } catch (Exception ignored) {
            limelight = null;
            connected = false;
            running = false;
            clearVisionData();
        }
    }

    public void update() {
        lastUpdateSeconds = getTimeSeconds();

        if (limelight == null) {
            clearVisionData();
            return;
        }

        updateConnectionState();

        if (!LIMELIGHT_ENABLED) {
            pause();
            clearVisionData();
            return;
        }

        if (!running) {
            start();
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            clearVisionData();
            return;
        }

        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

        if (ta < MIN_TARGET_AREA) {
            clearVisionData();
            return;
        }

        validTarget = true;

        lastValidTx = tx;
        lastValidTy = ty;
        lastValidTa = ta;

        rawCorrectionDeg = calculateRawCorrectionDegrees(tx);
        correctionDeg = clamp(
                rawCorrectionDeg,
                -MAX_CORRECTION_DEG,
                MAX_CORRECTION_DEG
        );

        visionLocked = Math.abs(tx) <= VISION_LOCK_TOLERANCE_DEG;
    }

    public void start() {
        if (limelight == null) {
            return;
        }

        limelight.start();
        updateConnectionState();
    }

    public void pause() {
        if (limelight == null) {
            return;
        }

        limelight.pause();
        updateConnectionState();
    }

    public void stop() {
        if (limelight == null) {
            return;
        }

        limelight.stop();
        updateConnectionState();
        clearVisionData();
    }

    public void setEnabled(boolean enabled) {
        LIMELIGHT_ENABLED = enabled;

        if (enabled) {
            start();
        } else {
            pause();
            clearVisionData();
        }
    }

    public void toggleEnabled() {
        setEnabled(!LIMELIGHT_ENABLED);
    }

    public boolean isEnabled() {
        return LIMELIGHT_ENABLED;
    }

    public boolean hasTarget() {
        return LIMELIGHT_ENABLED && validTarget;
    }

    public boolean isVisionLocked() {
        return LIMELIGHT_ENABLED && validTarget && visionLocked;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTargetArea() {
        return ta;
    }

    public double getTurretCorrectionDegrees() {
        if (!LIMELIGHT_ENABLED || !validTarget) {
            return 0.0;
        }

        return correctionDeg;
    }

    public double getTurretCorrectionRadians() {
        return Math.toRadians(getTurretCorrectionDegrees());
    }

    private double calculateRawCorrectionDegrees(double targetXDegrees) {
        double correctedTx = targetXDegrees;

        if (Math.abs(correctedTx) <= TX_DEADBAND_DEG) {
            correctedTx = 0.0;
        }

        return (correctedTx * CORRECTION_GAIN * CORRECTION_DIRECTION)
                + CAMERA_TO_SHOOTER_OFFSET_DEG;
    }

    private void updateConnectionState() {
        if (limelight == null) {
            connected = false;
            running = false;
            return;
        }

        connected = limelight.isConnected();
        running = limelight.isRunning();
    }

    private void clearVisionData() {
        validTarget = false;
        visionLocked = false;

        tx = 0.0;
        ty = 0.0;
        ta = 0.0;

        rawCorrectionDeg = 0.0;
        correctionDeg = 0.0;
    }

    private double getTimeSeconds() {
        return System.nanoTime() / 1_000_000_000.0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}