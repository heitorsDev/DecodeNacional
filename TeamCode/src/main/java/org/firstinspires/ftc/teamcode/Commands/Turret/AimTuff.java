
package org.firstinspires.ftc.teamcode.Commands.Turret;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;

import java.util.Timer;

public class AimTuff extends CommandBase {
    private Turret turret;
    public AimTuff(Turret turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(Turret.STATES.AIM);
    }

    @Override
    public boolean isFinished() {
        return turret.turretStatic();
    }
}

