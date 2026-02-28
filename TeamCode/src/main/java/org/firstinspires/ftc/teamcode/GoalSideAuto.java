package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
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

    PathChain firstRowChain;
    PathChain secondRowChain;
    PathChain thirdRowChain;

    Path gateIntakeToShoot;
    Path shootToGateIntake;
    public Pose shootingPose;
    public Pose shootingPose2;

    @Override
    public void initialize() {
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        gate.Open();
        TurretConstants.SIDES side = null;

        telemetry.addLine("Press X for blue side, B for red side: ");
        telemetry.update();
        while (side == null) {
            if (gamepad1.x) {
                side = TurretConstants.SIDES.BLUE;
            } else if (gamepad1.b) {
                side = TurretConstants.SIDES.RED;
            }
        }
        telemetry.update();

        PosePersistency.lastSide = side;
        turret.setSide(side);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        super.reset();

        follower = Constants.createFollower(hardwareMap);

        Pose startPose = new Pose(35.090, 132.823, Math.toRadians(-90));

        shootingPose = new Pose(58, 75, Math.toRadians(-170));
        shootingPose2 = new Pose(55, 100, Math.toRadians(-150));
        Pose startFirstRow = new Pose(40, 84, Math.toRadians(190));
        Pose endFirstRow   = new Pose(18, 84, Math.toRadians(190));

        Pose startSecondRow = new Pose(40, 63, Math.toRadians(190));
        Pose endSecondRow   = new Pose(14, 52, Math.toRadians(190));

        Pose startThirdRow = new Pose(40, 38, Math.toRadians(190));
        Pose endThirdRow   = new Pose(14, 38, Math.toRadians(190));

        Pose gateIntake = new Pose(12.1, 61.4, Math.toRadians(142));

        if (side == TurretConstants.SIDES.RED) {
            startPose = startPose.mirror();
            shootingPose2 = shootingPose2.mirror();
            shootingPose = shootingPose.mirror();
            startFirstRow = startFirstRow.mirror();
            endFirstRow = endFirstRow.mirror();
            startSecondRow = startSecondRow.mirror();
            endSecondRow = endSecondRow.mirror();
            startThirdRow = startThirdRow.mirror();
            endThirdRow = endThirdRow.mirror();
            gateIntake = gateIntake.mirror();
        }

        follower.setPose(startPose);

        startToShoot = new Path(new BezierLine(startPose, shootingPose2));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootingPose2.getHeading());

        // First row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartFirstRow = startFirstRow;
        Pose finalEndFirstRow = endFirstRow;
        firstRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose2, finalStartFirstRow))
                .setLinearHeadingInterpolation(shootingPose2.getHeading(), finalStartFirstRow.getHeading())
                .addPath(new BezierLine(finalStartFirstRow, finalEndFirstRow))
                .setLinearHeadingInterpolation(finalStartFirstRow.getHeading(), finalEndFirstRow.getHeading())
                .addPath(new BezierLine(finalEndFirstRow, shootingPose))
                .setLinearHeadingInterpolation(finalEndFirstRow.getHeading(), shootingPose.getHeading())
                .build();

        // Second row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartSecondRow = startSecondRow;
        Pose finalEndSecondRow = endSecondRow;
        secondRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, finalStartSecondRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), finalStartSecondRow.getHeading())
                .addPath(new BezierLine(finalStartSecondRow, finalEndSecondRow))
                .setLinearHeadingInterpolation(finalStartSecondRow.getHeading(), finalEndSecondRow.getHeading())
                .addPath(new BezierLine(finalEndSecondRow, shootingPose))
                .setLinearHeadingInterpolation(finalEndSecondRow.getHeading(), shootingPose.getHeading())
                .build();

        // Third row: shoot -> startRow -> endRow -> shoot, all in one chain
        Pose finalStartThirdRow = startThirdRow;
        Pose finalEndThirdRow = endThirdRow;
        thirdRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, finalStartThirdRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), finalStartThirdRow.getHeading())
                .addPath(new BezierLine(finalStartThirdRow, finalEndThirdRow))
                .setLinearHeadingInterpolation(finalStartThirdRow.getHeading(), finalEndThirdRow.getHeading())
                .addPath(new BezierLine(finalEndThirdRow, shootingPose))
                .setLinearHeadingInterpolation(finalEndThirdRow.getHeading(), shootingPose.getHeading())
                .build();

        gateIntakeToShoot = new Path(new BezierLine(gateIntake, shootingPose));
        gateIntakeToShoot.setLinearHeadingInterpolation(gateIntake.getHeading(), shootingPose.getHeading());

        shootToGateIntake = new Path(new BezierLine(shootingPose, gateIntake));
        shootToGateIntake.setLinearHeadingInterpolation(shootingPose.getHeading(), gateIntake.getHeading());

        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> PosePersistency.lastPose = follower.getPose()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, startToShoot),
                        new TransferSequence(intake, gate, turret),

                        // First row: robot drives shoot->row->sweep->shoot without stopping

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, firstRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        // Second row: same pattern
                        new IntakeOn(intake),
                        new FollowPathCommand(follower, secondRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        // Gate intake sequences (unchanged)
                        new FollowPathCommand(follower, shootToGateIntake),
                        new IntakeOn(intake),
                        new WaitCommand(1500),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, gateIntakeToShoot),
                        new TransferSequence(intake, gate, turret),

                        new FollowPathCommand(follower, shootToGateIntake),
                        new IntakeOn(intake),
                        new WaitCommand(1500),
                        new IntakeOff(intake),
                        new FollowPathCommand(follower, gateIntakeToShoot),
                        new TransferSequence(intake, gate, turret),


                        new IntakeOn(intake),
                        new FollowPathCommand(follower, thirdRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret)


                )
        );

        PosePersistency.turret = turret;
        register(turret, gate, intake);
        telemetry.addLine("Ready to run");
        telemetry.update();
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