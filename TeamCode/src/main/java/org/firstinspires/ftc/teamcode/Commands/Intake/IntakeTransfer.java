package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;

public class IntakeTransfer extends CommandBase {

    private final Intake intake;

    public IntakeTransfer(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.OnTransfer();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

