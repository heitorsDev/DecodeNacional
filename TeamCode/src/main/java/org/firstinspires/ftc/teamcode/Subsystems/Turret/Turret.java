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
import org.firstinspires.ftc.teamcode.core.hardware.TauraServo;

@Config
public class Turret extends SubsystemBase {
    public enum STATES{
        STATIC,
        AIM
    }
    STATES state = STATES.AIM;
    public static int tuningVelocity = 0;

    TauraServo Taura1;
    TauraServo Taura2;
    Servo headlight;
    DcMotorEx shooter1;
    DcMotorEx shooter2;

    InterpLUT velocityInterpolation = new InterpLUT();
    double minDistance = 58;
    double maxDistance = 145;

    Pose lastPose = new Pose(0, 0, 0);
    Pose botPose = new Pose(0, 0, 0);
    Pose poseToAim = new Pose(0, 0, 0);
    Pose virtualBotPose = new Pose(0, 0, 0);

    private Vector movementVector = new Vector(0, 0);
    double virtualBotMultiplier = 2;

    double distance = 0;
    double targetAngleFC = 0;

    public PIDController turretController = new PIDController(3, 0, 0.09);
    TurretConstants.SIDES side = TurretConstants.SIDES.BLUE;

    Telemetry telemetry;

    // ── Fix 1: no instance needed — RobotDrawer is used purely via static methods ──
    // Init is called explicitly in constructor via RobotDrawer.init()

    public Turret(HardwareMap hardwareMap) {

        headlight = hardwareMap.get(Servo.class, "headlight");
        telemetry = FtcDashboard.getInstance().getTelemetry();

        Taura1 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.HMTaura1));
        Taura2 = new TauraServo(hardwareMap.get(Servo.class, TurretConstants.HMTaura2));
        Taura1.setAnalogFeedbackSensor(hardwareMap.get(AnalogInput.class, TurretConstants.HMEncoder));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        reinitMotors();

        velocityInterpolation.add(minDistance, 840);
        velocityInterpolation.add(76, 920);
        velocityInterpolation.add(94, 990);
        velocityInterpolation.add(108, 1090);
        velocityInterpolation.add(maxDistance, 1255);
        velocityInterpolation.createLUT();
    }


    public void setSide(TurretConstants.SIDES side) {
        this.side = side;
    }

    public double getDistance() {
        return distance;
    }

    public double getTurretAngle() {
        double encoderAngle = Math.toRadians(-Taura1.getIncrementalPositionInDegrees() / (180.0 / 70.0));
        return normalizeAngle(encoderAngle);
    }

    public void updateBotPose(Pose pose) {
        this.lastPose = this.botPose;
        this.botPose = pose;

        movementVector = new Vector(
                lastPose.distanceFrom(botPose),
                Math.atan2(botPose.getY() - lastPose.getY(), botPose.getX() - lastPose.getX())
        ).times(virtualBotMultiplier);

        virtualBotPose = new Pose(
                botPose.getX() + movementVector.getXComponent(),
                botPose.getY() + movementVector.getYComponent(),
                botPose.getHeading()
        );
    }

    public void reinitMotors() {
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 100, 19.5));
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(400, 0, 100, 19.5));
    }
    public void setState(STATES state){
        this.state = state;
    }
    int targetVelocity = 0;
    public void setShooterVelocity(int power) {
        targetVelocity = power;
        shooter1.setVelocity(power);
        shooter2.setVelocity(power);
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Update target pose based on side
        switch (side) {
            case RED:
                this.poseToAim = TurretConstants.redGoalPose;
                break;
            case BLUE:
                this.poseToAim = TurretConstants.blueGoalPose;
                break;
        }

        updateTurret();
        updateShooter();

        // Fix 3: use static draw() — correct method signature, heading uses getHeadingAsUnitVector() inside RobotDrawer
        RobotDrawer.draw(botPose, "#3F51B5");       // actual pose  (blue)
        RobotDrawer.draw(virtualBotPose, "#E53935"); // virtual pose (red)
        RobotDrawer.draw(poseToAim, "#43A047");      // target goal  (green)


        telemetry.addData("Position: ", getTurretAngle());
        telemetry.addData("BOtpose: ", botPose);
        telemetry.addData("Virtual botpose: ", virtualBotPose);
        telemetry.addData("Distance: ", distance);
        telemetry.addData("Encoder Shooter1: ", shooter1.getVelocity());
        telemetry.addData("Encoder Shooter2: ", shooter2.getVelocity());
        telemetry.update();
    }

    // ── Private helpers ───────────────────────────────────────────────────────
    public boolean atVelocity(){
        int tolerance = 50;
        return Math.abs(this.targetVelocity-shooter1.getVelocity())<tolerance;
    }
    public boolean turretStatic(){
        return turretStatic;
    }
    private boolean turretStatic = false;
    private void updateTurret() {
        targetAngleFC = -Math.atan2(poseToAim.getY() - botPose.getY(), poseToAim.getX() - botPose.getX()) + Math.PI;
        double targetAngleRC = normalizeAngle(targetAngleFC + botPose.getHeading());

        if (this.side == TurretConstants.SIDES.RED) {
            targetAngleRC += TurretConstants.redOffset;
        } else {
            targetAngleRC += TurretConstants.blueOffset;
        }

        turretStatic = targetAngleRC > -Math.toRadians(120)
                && targetAngleRC < Math.toRadians(120)
                && Math.abs(turretController.getPositionError()) < Math.toRadians(10);

        headlight.setPosition(turretStatic ? 1 : 0);
        switch (state){
            case AIM:
                targetAngleRC = Range.clip(targetAngleRC, -Math.toRadians(120), Math.toRadians(120));
                break;
            case STATIC:
                targetAngleRC = 0;
                break;
        }


        double currentAngle = getTurretAngle();
        double power = Range.clip(
                turretController.calculate(currentAngle, targetAngleRC) / 2,
                -0.5, 0.5
        );

        Taura1.setPosition(0.5 + power);
        Taura2.setPosition(0.5 + power);

        distance = botPose.distanceFrom(poseToAim);
    }

    private void updateShooter() {
        setShooterVelocity(
                (int) velocityInterpolation.get(Range.clip(distance, minDistance + 1, maxDistance - 1))
        );
        //setShooterVelocity(tuningVelocity);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}