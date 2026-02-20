package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;

public class PosePersistency {
    public static Pose lastPose = new Pose(0,0,0);
    public static TurretConstants.SIDES lastSide = TurretConstants.SIDES.BLUE;
}
