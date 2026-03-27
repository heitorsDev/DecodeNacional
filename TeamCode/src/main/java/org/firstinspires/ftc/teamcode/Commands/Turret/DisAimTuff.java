
package org.firstinspires.ftc.teamcode.Commands.Turret;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;

public class DisAimTuff extends CommandBase {
    private Turret turret;

    public DisAimTuff(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(Turret.STATES.STATIC);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}

