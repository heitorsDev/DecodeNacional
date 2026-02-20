package org.firstinspires.ftc.teamcode.Subsystems.Turret.Vision;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final Limelight3A limelight;

    public VisionSubsystem(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                VisionConstants.LIMELIGHT_NAME
        );

        // Seleciona pipeline MT2
        limelight.pipelineSwitch(VisionConstants.MT2_PIPELINE);
    }

    public Optional<Pose> getRobotPoseMT2(double fallbackHeading) {

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Optional.empty();
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return Optional.empty();
        }

        double xMeters = botpose.getPosition().x;
        double yMeters = botpose.getPosition().y;
        double xInches = xMeters * VisionConstants.METERS_TO_INCHES;
        double yInches = yMeters * VisionConstants.METERS_TO_INCHES;

        return Optional.of(
                new Pose(
                        xInches,
                        yInches,
                        fallbackHeading // IGNORA yaw da câmera
                )
        );
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getBotpose() != null;
    }
}