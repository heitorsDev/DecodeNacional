package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.geometry.Pose;
import com.acmerobotics.dashboard.config.Config;

public class RobotDrawer {

    public static final double ROBOT_RADIUS = 9;

    private static final FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * Draw robot on FTC Dashboard field overlay
     */
    public static void draw(Pose pose, String color) {
        if (isInvalid(pose)) return;

        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Draw robot body
        canvas.setStroke(color);
        canvas.strokeCircle(x, y, ROBOT_RADIUS);

        // Draw heading line
        double headingX = x + Math.cos(heading) * ROBOT_RADIUS;
        double headingY = y + Math.sin(heading) * ROBOT_RADIUS;

        canvas.strokeLine(x, y, headingX, headingY);

        dashboard.sendTelemetryPacket(packet);
    }

    private static boolean isInvalid(Pose pose) {
        return Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading());
    }
}