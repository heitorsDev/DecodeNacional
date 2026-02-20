package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {


    public static final String HMTaura1 = "Taura1";
    public static final String HMTaura2 = "Taura2";

    public static final Pose blueGoalPose = new Pose(144,0,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();
    public static enum SIDES{
        BLUE,
        RED
    }
}

