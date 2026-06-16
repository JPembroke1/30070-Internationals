package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.geometry.Pose;

public class PoseStorage {

    public static Pose currentPose = null;

    public static void clear() {
        currentPose = null;
    }

    public static void setPose(Pose pose) {
        if (pose == null) {
            currentPose = null;
        } else {
            currentPose = new Pose(
                    pose.getX(),
                    pose.getY(),
                    pose.getHeading()
            );
        }
    }

    public static Pose getPose() {
        if (currentPose == null) return null;

        return new Pose(
                currentPose.getX(),
                currentPose.getY(),
                currentPose.getHeading()
        );
    }
}