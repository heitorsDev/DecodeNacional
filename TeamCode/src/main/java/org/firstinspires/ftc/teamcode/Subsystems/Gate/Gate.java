package org.firstinspires.ftc.teamcode.Subsystems.Gate;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.Subsystems.Gate.GateConstants;
@Config

public class Gate extends SubsystemBase {
    Servo gate1;
    public static double position = 0;
    Servo gate2;
    public Gate(HardwareMap hardwareMap){
        gate1 = hardwareMap.get(Servo.class, GateConstants.HMGate1);
       // gate2 = hardwareMap.get(Servo.class, GateConstants.HMGate2);
    }
    public void Open(){
        gate1.setPosition(GateConstants.OpenPosition);
      //  gate2.setPosition(-GateConstants.OpenPosition);
    }
    public void Close(){
        gate1.setPosition(GateConstants.ClosedPosition);
      //  gate2.setPosition(-GateConstants.ClosedPosition);
    }

    @Override
    public void periodic(){
        gate1.setPosition(position);
    }
}
