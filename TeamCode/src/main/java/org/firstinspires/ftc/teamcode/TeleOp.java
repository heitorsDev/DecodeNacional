package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.TransferSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Vision.Vision;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Optional;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends CommandOpMode {
    private Turret turret;
    private Gate gate;
    private Intake intake;
    private Follower follower;
    private Vision vision;


    @Override
    public void initialize() {

        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        vision = new Vision(hardwareMap);

        turret.setSide(PosePersistency.lastSide);


        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new IntakeOn(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenActive(new IntakeOff(intake));
        gamepadEx.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenActive(new TransferSequence(intake, gate, turret));


        register(turret, gate,intake, vision);



        follower.startTeleOpDrive();
        
    }
    @Override
    public void run(){
        super.run();
        Optional <Pose> optionalPose = vision.getVisionPose(follower.getPose());
        Pose visionPose = (optionalPose.orElse(null));
        if (visionPose!=null){
            //follower.setPose(visionPose);
            telemetry.addData("Vision pose: ", visionPose.toString());
        }
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        turret.updateBotPose(follower.getPose());
        telemetry.update();
    }
}
