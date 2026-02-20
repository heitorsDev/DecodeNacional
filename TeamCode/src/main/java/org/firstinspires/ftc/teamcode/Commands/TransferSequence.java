package org.firstinspires.ftc.teamcode.Commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Commands.Gate.CloseGate;
import org.firstinspires.ftc.teamcode.Commands.Gate.OpenGate;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOff;
import org.firstinspires.ftc.teamcode.Commands.Intake.IntakeOn;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;

public class TransferSequence extends SequentialCommandGroup {


    public TransferSequence(Intake intake, Gate gate, Turret turret){



        addCommands(

                new IntakeOn(intake),
                new CloseGate(gate),
                new WaitCommand(1500),
                new OpenGate(gate),
                new IntakeOff(intake)

        );
        addRequirements(intake, gate);
    }

}
