package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public class RobotDrawer {

    public static final double ROBOT_RADIUS = 9.0;
    public static final String DEFAULT_COLOR = "#3F51B5";

    private final String color;
    private static FieldManager field;

    public RobotDrawer(String color) {
        this.color = color;
    }

    public RobotDrawer() {
        this(DEFAULT_COLOR);
    }

    /** Call once in your OpMode or Subsystem constructor. */
    public static void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /** Instance draw — uses this instance's color. */
    public void draw(Pose pose) {
        draw(pose, this.color);
    }

    /** Static draw — use this to draw multiple poses with different colors. */
    public static void draw(Pose pose, String color) {
        if (field == null || isInvalid(pose)) return;

        Style style = new Style("", color, 0.0);

        // Draw robot body circle
        field.setStyle(style);
        field.moveCursor(pose.getX(), pose.getY());
        field.circle(ROBOT_RADIUS);

        // Draw heading indicator using getHeadingAsUnitVector() — fixes coordinate system mismatch
        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);

        double x1 = pose.getX() + v.getXComponent() / 2;
        double y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent();
        double y2 = pose.getY() + v.getYComponent();

        field.setStyle(style);
        field.moveCursor(x1, y1);
        field.line(x2, y2);
    }

    /** Must be called once per loop AFTER all draw() calls to flush to Panels. */
    public static void update() {
        if (field != null) field.update();
    }

    private static boolean isInvalid(Pose pose) {
        return Double.isNaN(pose.getX())
                || Double.isNaN(pose.getY())
                || Double.isNaN(pose.getHeading());
    }
}