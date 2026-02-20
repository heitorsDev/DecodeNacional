package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.TransferSequence;
import org.firstinspires.ftc.teamcode.Field.Sides;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Goal Side Auto")
public class GoalSideAuto extends CommandOpMode {
    Turret turret;
    Intake intake;
    Gate gate;

    Follower follower;


    Path startToShoot;

    Path shootToFirstRow;
    Path firstRowSweep;
    Path firstRowToShoot;

    Path shootToSecondRow;
    Path secondRowSweep;
    Path secondRowToShoot;

    Path shootToThirdRow;
    Path thirdRowSweep;
    Path thirdRowToShoot;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        gate.Open();
        TurretConstants.SIDES side = null;

        telemetry.addLine("Press X for blue side, B for red side: ");
        while(side==null){
            if (gamepad1.x){
                side = TurretConstants.SIDES.BLUE;
            } else
            if (gamepad1.b){
                side = TurretConstants.SIDES.RED;
            }
        }
        telemetry.update();

        PosePersistency.lastSide =side;
        turret.setSide(side);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(35.090, 132.823, Math.toRadians(-90));
        Pose shootingPose = new Pose(50, 100, Math.toRadians(-90));

        Pose startFirstRow = new Pose(40, 85, Math.toRadians(180));
        Pose endFirstRow   = new Pose(16, 85, Math.toRadians(180));

        Pose startSecondRow = new Pose(40, 50, Math.toRadians(180));
        Pose endSecondRow   = new Pose(16, 50, Math.toRadians(180));

        Pose startThirdRow = new Pose(40, 36, Math.toRadians(180));
        Pose endThirdRow   = new Pose(16, 36, Math.toRadians(180));

        if (side== TurretConstants.SIDES.RED){
            startPose = startPose.mirror();
            shootingPose = shootingPose.mirror();
            startFirstRow = startFirstRow.mirror();
            endFirstRow = endFirstRow.mirror();
            startSecondRow = startSecondRow.mirror();
            endSecondRow = endSecondRow.mirror();
            startThirdRow = startThirdRow.mirror();
            endThirdRow = endThirdRow.mirror();
        }


        follower.setPose(startPose);


        startToShoot = new Path(
                new BezierLine(startPose, shootingPose)
        );
        startToShoot.setLinearHeadingInterpolation(
                startPose.getHeading(),
                shootingPose.getHeading()
        );

        shootToFirstRow = new Path(
                new BezierLine(shootingPose, startFirstRow)
        );
        shootToFirstRow.setLinearHeadingInterpolation(
                shootingPose.getHeading(),
                startFirstRow.getHeading()
        );

        firstRowSweep = new Path(
                new BezierLine(startFirstRow, endFirstRow)
        );
        firstRowSweep.setLinearHeadingInterpolation(
                startFirstRow.getHeading(),
                endFirstRow.getHeading()
        );

        firstRowToShoot = new Path(
                new BezierLine(endFirstRow, shootingPose)
        );
        firstRowToShoot.setLinearHeadingInterpolation(
                endFirstRow.getHeading(),
                shootingPose.getHeading()
        );

        shootToSecondRow = new Path(
                new BezierLine(shootingPose, startSecondRow)
        );
        shootToSecondRow.setLinearHeadingInterpolation(
                shootingPose.getHeading(),
                startSecondRow.getHeading()
        );

        secondRowSweep = new Path(
                new BezierLine(startSecondRow, endSecondRow)
        );
        secondRowSweep.setLinearHeadingInterpolation(
                startSecondRow.getHeading(),
                endSecondRow.getHeading()
        );

        secondRowToShoot = new Path(
                new BezierLine(endSecondRow, shootingPose)
        );
        secondRowToShoot.setLinearHeadingInterpolation(
                endSecondRow.getHeading(),
                shootingPose.getHeading()
        );

        shootToThirdRow = new Path(
                new BezierLine(shootingPose, startThirdRow)
        );
        shootToThirdRow.setLinearHeadingInterpolation(
                shootingPose.getHeading(),
                startThirdRow.getHeading()
        );

        thirdRowSweep = new Path(
                new BezierLine(startThirdRow, endThirdRow)
        );
        thirdRowSweep.setLinearHeadingInterpolation(
                startThirdRow.getHeading(),
                endThirdRow.getHeading()
        );

        thirdRowToShoot = new Path(
                new BezierLine(endThirdRow, shootingPose)
        );
        thirdRowToShoot.setLinearHeadingInterpolation(
                endThirdRow.getHeading(),
                shootingPose.getHeading()
        );

        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(()->PosePersistency.lastPose=follower.getPose()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, startToShoot),
                        new TransferSequence(intake, gate, turret),
                        new FollowPathCommand(follower, shootToFirstRow),
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, firstRowSweep),
                        new FollowPathCommand(follower, firstRowToShoot),
                        new TransferSequence(intake, gate, turret),

                        new FollowPathCommand(follower, shootToSecondRow),
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, secondRowSweep),
                        new FollowPathCommand(follower, secondRowToShoot),
                        new TransferSequence(intake, gate, turret),

                        new FollowPathCommand(follower, shootToThirdRow),
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, thirdRowSweep),
                        new FollowPathCommand(follower, thirdRowToShoot),
                        new TransferSequence(intake, gate, turret)
                )
        );
        register(turret, gate, intake);
    }

    @Override
    public void run() {
        PosePersistency.lastPose = follower.getPose();
        turret.updateBotPose(follower.getPose());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
        super.run();
    }
}
