package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.pedropathing.geometry.Pose;

public class TurretConstants {


    public static final String HMTaura1 = "turret1";
    public static final String HMTaura2 = "turret2";

    public static final String HMEncoder = "potentiometer";

    public static final Pose blueGoalPose = new Pose(0,144,0);
    public static final Pose redGoalPose = blueGoalPose.mirror();

    public static double redOffset = -Math.toRadians(10);
    public static double blueOffset = Math.toRadians(2);
    public static enum SIDES{
        BLUE,
        RED
    }
}

