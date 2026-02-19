package org.firstinspires.ftc.teamcode.Subsystems.Intake;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Intake extends SubsystemBase {

    DcMotor Intake1;
    DcMotor Intake2;
    public Intake(HardwareMap hardwareMap) {
        Intake1 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake1);
        Intake2 = hardwareMap.get(DcMotor.class, IntakeConstants.HMIntake2);
    }
public void On(){
    Intake1.setPower(IntakeConstants.TurnOn);
    Intake2.setPower(-IntakeConstants.TurnOn);
}
public void Off(){
    Intake1.setPower(IntakeConstants.TurnOff);
    Intake2.setPower(-IntakeConstants.TurnOff);
}

@Override
public void periodic(){

}

}
