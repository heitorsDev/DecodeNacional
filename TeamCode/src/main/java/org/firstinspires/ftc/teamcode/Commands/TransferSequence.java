package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Gate.CloseGate;
import org.firstinspires.ftc.teamcode.Commands.Gate.OpenGate;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeTransfer;
import org.firstinspires.ftc.teamcode.Commands.Turret.AimTuff;
import org.firstinspires.ftc.teamcode.Commands.Turret.DisAimTuff;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;

public class TransferSequence extends SequentialCommandGroup {


    public TransferSequence(Intake intake, Gate gate, Turret turret){



        addCommands(
                new IntakeOff(intake),
                new CloseGate(gate),
                new WaitCommand(200),
                new IntakeTransfer(intake),

                new WaitCommand(700),
                new OpenGate(gate),
                new IntakeOff(intake)

        );
        addRequirements(intake, gate);
    }

}
