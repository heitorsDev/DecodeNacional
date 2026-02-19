package org.firstinspires.ftc.teamcode.Commands.Intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;

public class IntakeOn extends CommandBase {

    private final Intake intake;

    public IntakeOn(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.On();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

