package org.firstinspires.ftc.teamcode.Commands;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.PosePersistency;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.Vision.Vision;

public class VisionResetOffset extends CommandBase {
    Limelight3A limelight3A;
    Turret turret;
    public VisionResetOffset(HardwareMap hMap, Turret turret){
        this.turret = turret;
        limelight3A = hMap.get(Limelight3A.class, "limelight");
        limelight3A.start();
        limelight3A.pipelineSwitch(0);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        LLResult lastResult = limelight3A.getLatestResult();
        if (lastResult != null){
            double turretAngleBC = turret.getTurretAngle();
            double tx = lastResult.getTx();
            double delta = tx-turretAngleBC;
            switch (PosePersistency.lastSide){
                case RED:
                    TurretConstants.blueOffset +=delta;
                    break;
                case BLUE:
                    TurretConstants.redOffset -=delta;
                    break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
