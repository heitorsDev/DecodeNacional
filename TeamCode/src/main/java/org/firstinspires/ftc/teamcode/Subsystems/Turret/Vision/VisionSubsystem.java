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
        limelight.pipelineSwitch(VisionConstants.MT2_PIPELINE);
    }

    public void updateHeading(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    }

    public Optional<Pose> getVisionPose(double currentHeadingRadians) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return Optional.empty();

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return Optional.empty();

        double xInches = botpose.getPosition().x * VisionConstants.METERS_TO_INCHES;
        double yInches = botpose.getPosition().y * VisionConstants.METERS_TO_INCHES;

        return Optional.of(new Pose(xInches, yInches, currentHeadingRadians));
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getBotpose_MT2() != null;
    }
}