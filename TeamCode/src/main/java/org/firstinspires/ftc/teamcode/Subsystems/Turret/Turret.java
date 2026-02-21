package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import  org.firstinspires.ftc.teamcode.core.hardware.TauraServo;
@Config
public class Turret extends SubsystemBase {
    public static int tuningVelocity = 0;
    TauraServo Taura1;
    TauraServo Taura2;

    DcMotorEx shooter1;
    DcMotorEx shooter2;

    InterpLUT velocityInterpolation = new InterpLUT();
    double minDistance = 0;
    double maxDistance = 1000;



    PIDController turretController = new PIDController(0.2,0,0);
    Pose botPose = new Pose(0,0,0);
    Pose poseToAim = new Pose(0,0,0);
    private double getTurretAngle(){
        return Math.toRadians(Taura1.getRawPositionInDegrees());
    }
    public boolean okToShoot = false;
    public void updateBotPose(Pose pose){
        this.botPose = pose;
    }
    TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;
    public void setSide(TurretConstants.SIDES side){
        this.side = side;
    }

    double distance = 0;

    private void updateTurret(){
        double goalAngleFC = Math.atan2(poseToAim.getY()-botPose.getY(),poseToAim.getX()-botPose.getX());
        double goalAngleBC = goalAngleFC-botPose.getHeading();//BC stands for bot-centric, FC for field-centric

        double goalAngleBCcorrected = Range.clip(goalAngleBC,Math.toRadians(-90),Math.toRadians(90));
        okToShoot = goalAngleBCcorrected==goalAngleBC;
        if (!okToShoot){
            goalAngleBCcorrected=0;
        }
        turretController.setSetPoint(goalAngleBCcorrected);
        double power = turretController.calculate(getTurretAngle())/2;
        Taura1.setPosition(
            0.5+power
        );
        Taura2.setPosition(
                0.5+power
        );
        distance = botPose.distanceFrom(poseToAim);



    }
    public void setShooterVelocity(int power){
        shooter1.setVelocity(power);
        shooter2.setVelocity(power);
    }
    private void updateShooter(){
        setShooterVelocity(
                (int) velocityInterpolation.get(Range.clip(distance,minDistance+1, maxDistance -1)
        ));
    }
    Telemetry telemetry;
    public Turret(HardwareMap hardwareMap){
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Taura1 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.HMTaura1));
        Taura2 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.HMTaura2));
        Taura1.setAnalogFeedbackSensor(hardwareMap.get(AnalogInput.class, TurretConstants.HMEncoder));
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
    }
    
    @Override
    public void periodic(){
        switch (side){
            case RED:
                this.poseToAim = TurretConstants.redGoalPose;
                break;
            case BLUE:
                this.poseToAim = TurretConstants.blueGoalPose;
                break;
        }
        updateTurret();
        setShooterVelocity(tuningVelocity);
        telemetry.addData("Distance: ", distance);
        telemetry.update();

    }

}

