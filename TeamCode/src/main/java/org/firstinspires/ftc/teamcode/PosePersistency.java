package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;

public class PosePersistency {
    public static Turret turret = null;
    public static Pose lastPose = new Pose(0,0,0);
    public static TurretConstants.SIDES lastSide = TurretConstants.SIDES.RED;


    private static final Pose RED_RESET_POSE = new Pose(6.625, 8.635, Math.toRadians(0));

    public static Pose getResetPose() {
        if (lastSide == TurretConstants.SIDES.BLUE) {
            return RED_RESET_POSE.mirror();
        }
        return RED_RESET_POSE;
    }

    public static void applyReset(Follower follower) {
        Pose resetPose = getResetPose();
        lastPose = resetPose;
        follower.setPose(resetPose);
    }
}

