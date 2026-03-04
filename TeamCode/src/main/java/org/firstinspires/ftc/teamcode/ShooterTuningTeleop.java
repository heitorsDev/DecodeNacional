package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.Intake;

@Config
@TeleOp(name = "Shooter PIDF Tuning", group = "Tuning")
public class ShooterTuningTeleop extends LinearOpMode {

    // --- Configurable via FTC Dashboard ---
    public static double PIDF_P = 50;
    public static double PIDF_I = 0;
    public static double PIDF_D = 0;
    public static double PIDF_F = 30;

    public static int TARGET_VELOCITY = 0;

    // ----------------------------------------

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        configureMotors();

        telemetry.addLine("Shooter PIDF Tuning Ready.");
        telemetry.addLine("Adjust PIDF_P/I/D/F and TARGET_VELOCITY on the Dashboard.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intake.periodic();
            if (gamepad1.a){
                intake.OnTransfer();
            } else {
                intake.Off();
            }
            // Re-apply PIDF coefficients every loop so Dashboard changes take effect live
            configureMotors();

            shooter1.setVelocity(TARGET_VELOCITY);
            shooter2.setVelocity(TARGET_VELOCITY);

            double vel1 = shooter1.getVelocity();
            double vel2 = shooter2.getVelocity();

            telemetry.addLine("=== Shooter Tuning ===");
            telemetry.addData("Target Velocity (ticks/s)", TARGET_VELOCITY);
            telemetry.addLine("--- Shooter 1 ---");
            telemetry.addData("  Actual Velocity", String.format("%.1f", vel1));
            telemetry.addData("  Error", String.format("%.1f", TARGET_VELOCITY - vel1));
            telemetry.addLine("--- Shooter 2 ---");
            telemetry.addData("  Actual Velocity", String.format("%.1f", vel2));
            telemetry.addData("  Error", String.format("%.1f", TARGET_VELOCITY - vel2));
            telemetry.addLine("--- PIDF (active) ---");
            telemetry.addData("  P", PIDF_P);
            telemetry.addData("  I", PIDF_I);
            telemetry.addData("  D", PIDF_D);
            telemetry.addData("  F", PIDF_F);
            telemetry.update();
        }

        // Stop motors when OpMode ends
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }

    private void configureMotors() {
        PIDFCoefficients pidf = new PIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }
}