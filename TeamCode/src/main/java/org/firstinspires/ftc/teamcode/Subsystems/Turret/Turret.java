package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import  org.firstinspires.ftc.teamcode.core.hardware.TauraServo;

public class Turret extends SubsystemBase {

TauraServo Taura1;
TauraServo Taura2;
    public Turret(HardwareMap hardwareMap){
        Taura1  = hardwareMap.get(Servo.class, TurretConstants.HMTaura1);
        Taura2 = hardwareMap.get(Servo.class, TurretConstants.HMTaura2);
    }
    
    @Override
    public void periodic(){

    }
}

