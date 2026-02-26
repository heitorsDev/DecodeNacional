package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    double minDistance = 60;
    double maxDistance = 123;
    Pose lastPose = new Pose(0,0,0);


    PIDController turretController = new PIDController(1,0,0);
    Pose botPose = new Pose(0,0,0);
    Pose poseToAim = new Pose(0,0,0);
    public double getTurretAngle(){
        double encoderAngle = Math.toRadians(-Taura1.getIncrementalPositionInDegrees() / (180.0/70.0));
        return normalizeAngle(encoderAngle);
    }
    public boolean okToShoot = false;
    Pose virtualBotPose = new Pose(0,0,0);
    private Vector movementVector = new Vector(0,0);
    double virtualBotMultiplier = 2;
    public void updateBotPose(Pose pose){
        this.lastPose = this.botPose;
        this.botPose = pose;
        movementVector = new Vector(
                lastPose.distanceFrom(botPose),
                Math.atan2(botPose.getY()-lastPose.getY(), botPose.getX()-lastPose.getX())
        ).times(virtualBotMultiplier);
        virtualBotPose = new Pose(
                botPose.getX()+movementVector.getXComponent(),
                botPose.getY()+movementVector.getYComponent(),
                botPose.getHeading()
        );


    }
    TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;
    public void setSide(TurretConstants.SIDES side){
        this.side = side;
    }

    double distance = 0;

    double targetAngleFC = 0; // input em radianos, field centric
    private void updateTurret(){
        targetAngleFC = -Math.atan2(poseToAim.getY()-botPose.getY(), poseToAim.getX()-botPose.getX())+Math.PI;
        double targetAngleRC = normalizeAngle(targetAngleFC + botPose.getHeading());

        targetAngleRC = Range.clip(targetAngleRC, -Math.PI/2, Math.PI/2);

        double currentAngle = getTurretAngle();

        double power = Range.clip(
                turretController.calculate(currentAngle, targetAngleRC) / 2,
                -0.5, 0.5
        );

        Taura1.setPosition(0.5 + power);
        Taura2.setPosition(0.5 + power);

        distance = virtualBotPose.distanceFrom(poseToAim);
    }
    private double normalizeAngle(double angle){
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
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
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0, 20));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 0, 20));

        velocityInterpolation.add(minDistance, 750);
        velocityInterpolation.add(81, 850);
        velocityInterpolation.add(94, 950);
        velocityInterpolation.add(maxDistance, 1050);
        velocityInterpolation.createLUT();

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
        updateShooter();
        //setShooterVelocity(tuningVelocity);
        telemetry.addData("Position: ", getTurretAngle());
        telemetry.addData("Distance: ", distance);
        telemetry.update();

    }

}

