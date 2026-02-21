package org.firstinspires.ftc.teamcode.Subsystems.Turret.Vision;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;


import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Optional;
public class Vision extends SubsystemBase {

    private final Limelight3A limelight;

    // Independent filter per axis.
    // Q (processNoise) and R (measurementNoise) are the two knobs to tune:
    //   - Raise Q if the robot moves fast or unpredictably
    //   - Raise R if vision readings are noisy / flickery
    //   - Lower R if you trust the Limelight more than odometry
    private final KalmanFilter1D xFilter = new KalmanFilter1D(0, 100, 0.1, 2.0);
    private final KalmanFilter1D yFilter = new KalmanFilter1D(0, 100, 0.1, 2.0);

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        limelight.pipelineSwitch(VisionConstants.MT2_PIPELINE);
        limelight.start();
    }

    public void updateHeading(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
    }

    public Optional<Pose> getVisionPose(Pose currentBotPose) {
        updateHeading(Math.toDegrees(currentBotPose.getHeading()));

        xFilter.predict();
        yFilter.predict();

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return Optional.empty();

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null) return Optional.empty();

        double rawX = botpose.getPosition().x * VisionConstants.METERS_TO_INCHES;
        double rawY = botpose.getPosition().y * VisionConstants.METERS_TO_INCHES;

        // Feed raw vision reading into each filter and get smoothed output
        double filteredX = xFilter.update(rawX);
        double filteredY = yFilter.update(rawY);

        return Optional.of(new Pose(filteredX, filteredY, currentBotPose.getHeading()));
    }

    public boolean hasValidTarget() {
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid() && result.getBotpose_MT2() != null;
    }
}