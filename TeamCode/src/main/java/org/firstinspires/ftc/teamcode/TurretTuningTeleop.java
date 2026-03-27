package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class TurretTuningTeleop extends LinearOpMode {

    // --- Configurável via FTC Dashboard ---
    public static double PID_P = 3;
    public static double PID_I = 0;
    public static double PID_D = 0;

    public static boolean USE_AIM = false; // true = AIM, false = STATIC
    // ----------------------------------------

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Follower follower = Constants.createFollower(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        turret.setState(Turret.STATES.STATIC);

        telemetry.addLine("Turret Tuning Ready.");
        telemetry.addLine("USE_AIM = false → STATIC (trava em 0°)");
        telemetry.addLine("USE_AIM = true  → AIM (aponta para o goal)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            turret.updateBotPose(follower.getPose());

            // Aplica PID atualizado do dashboard
            turret.turretController.setPID(PID_P, PID_I, PID_D);

            // Troca de estado via dashboard em tempo real
            turret.setState(USE_AIM ? Turret.STATES.AIM : Turret.STATES.STATIC);

            turret.periodic();

            double currentAngle = turret.getTurretAngle();

            telemetry.addLine("=== Turret Tuning ===");
            telemetry.addData("Estado", USE_AIM ? "AIM" : "STATIC");
            telemetry.addData("Ângulo Atual (graus)", String.format("%.2f", Math.toDegrees(currentAngle)));
            telemetry.addData("Distância ao Goal (cm)", String.format("%.1f", turret.getDistance()));
            telemetry.addData("Turret Estática?", turret.turretStatic());
            telemetry.addLine("--- PID (ativo) ---");
            telemetry.addData("  P", PID_P);
            telemetry.addData("  I", PID_I);
            telemetry.addData("  D", PID_D);
            telemetry.update();
        }
    }
}