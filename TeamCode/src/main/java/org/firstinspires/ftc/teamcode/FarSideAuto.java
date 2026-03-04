package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.TransferSequence;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Far Side Auto")
public class FarSideAuto extends CommandOpMode {
    Turret turret;
    Intake intake;
    Gate gate;

    Follower follower;

    Path startToShoot;

    PathChain thirdRowChain;
    PathChain hpChain;

    public Pose shootingPose;

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

        Pose startPose      = new Pose((144 / 2) - (17.5 / 2), 9.5, Math.toRadians(-90));
        shootingPose        = new Pose(58, 78, Math.toRadians(-170));

        Pose startThirdRow  = new Pose(50, 38, Math.toRadians(190));
        Pose endThirdRow    = new Pose(14, 38, Math.toRadians(190));

        Pose hpIntake           = new Pose(9, 9, Math.toRadians(-135));
        Pose hpControlToIntake  = new Pose(60, 50, 0);


        if (side == TurretConstants.SIDES.RED) {
            startPose           = startPose.mirror();
            shootingPose        = shootingPose.mirror();
            startThirdRow       = startThirdRow.mirror();
            endThirdRow         = endThirdRow.mirror();
            hpIntake            = hpIntake.mirror();
            hpControlToIntake   = hpControlToIntake.mirror();
        }

        follower.setPose(startPose);

        startToShoot = new Path(new BezierLine(startPose, shootingPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading());

        thirdRowChain = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, startThirdRow))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), startThirdRow.getHeading())
                .addPath(new BezierLine(startThirdRow, endThirdRow))
                .setLinearHeadingInterpolation(startThirdRow.getHeading(), endThirdRow.getHeading())
                .addPath(new BezierLine(endThirdRow, shootingPose))
                .setLinearHeadingInterpolation(endThirdRow.getHeading(), shootingPose.getHeading())
                .build();

        // Single chain: shoot -> HP intake -> shoot, both legs as Bezier curves
        hpChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                       shootingPose, hpControlToIntake, hpIntake))
                .addPath(new BezierCurve(
                        hpIntake,
                        hpControlToIntake,
                        shootingPose))
                .setReversed()
                .build();

        schedule(
                new RunCommand(() -> follower.update()),
                new RunCommand(() -> PosePersistency.lastPose = follower.getPose()),
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, startToShoot),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, thirdRowChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, hpChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, hpChain),
                        new IntakeOff(intake),
                        new TransferSequence(intake, gate, turret),

                        new IntakeOn(intake),
                        new FollowPathCommand(follower, hpChain),
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
        telemetry.addData("Distance", turret.getDistance());
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("Pose", follower.getPose());
        telemetry.update();
        super.run();
    }
}