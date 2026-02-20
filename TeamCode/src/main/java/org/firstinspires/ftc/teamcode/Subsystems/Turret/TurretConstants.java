package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {


    public static final String HMTaura1 = "turret1";
    public static final String HMTaura2 = "turret2";

    public static final String HMEncoder = "encoder";

    public static final Pose blueGoalPose = new Pose(144,0,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();


    public static enum SIDES{
        BLUE,
        RED
    }
}

