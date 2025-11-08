package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Motor Test", group = "Testing")
public class PIDMotorTest extends LinearOpMode {

    // Define the motor's ticks per revolution. This is crucial for correct velocity calculation.
    // For a goBILDA 5203 series motor, it's 28 ticks per revolution on the motor shaft.
    // The REV Hub multiplies this by 4 for quadrature encoding, resulting in 112.
    private static final double TICKS_PER_REV = 112.0;

    @Override
    public void runOpMode() {

        final ElapsedTime runtime = new ElapsedTime();

        // ===================== MOTOR DECLARATIONS =====================
        DcMotorEx launcherMotor = null;
        DcMotorEx intakeMotor = null;
        DcMotorEx kickerMotor = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = 500.0;
        final double kickerTargetRPM = 200.0;
        final double launcherTargetRPM = 2000.0;

        // Convert RPM targets to ticks per second
        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV;
        final double kickerVelocity = (kickerTargetRPM / 60.0) * TICKS_PER_REV;
        final double launcherVelocity = (launcherTargetRPM / 60.0) * TICKS_PER_REV;

        // ===================== INITIALIZATION =====================
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // Set PIDF coefficients. These may require tuning.
            launcherMotor.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 0.0);
            telemetry.addData("Launcher Motor", "✅ Connected");
        } catch (Exception e) {
            launcherMotor = null;
            telemetry.addData("Launcher Motor", "❌ Not found");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 0.0);
            telemetry.addData("Intake Motor", "✅ Connected");
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addData("Intake Motor", "❌ Not found");
        }

        try {
            kickerMotor = hardwareMap.get(DcMotorEx.class, "KickerMotor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerMotor.setVelocityPIDFCoefficients(10.0, 3.0, 0.0, 0.0);
            telemetry.addData("Kicker Motor", "✅ Connected");
        } catch (Exception e) {
            kickerMotor = null;
            telemetry.addData("Kicker Motor", "❌ Not found");
        }

        telemetry.addData("Status", "PID Motor Testing Ready");
        telemetry.addData("A", "Intake Motor (%.0f RPM)", intakeTargetRPM);
        telemetry.addData("B", "Kicker Motor (%.0f RPM)", kickerTargetRPM);
        telemetry.addData("Y", "Launcher Motor (%.0f RPM)", launcherTargetRPM);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ===================== MAIN CONTROL LOOP =====================
        while (opModeIsActive()) {

            // A Button: Intake Motor
            if (intakeMotor != null) {
                if (gamepad1.a) {
                    intakeMotor.setVelocity(intakeVelocity);
                } else {
                    intakeMotor.setPower(0);
                }
            }

            // B Button: Kicker Motor
            if (kickerMotor != null) {
                if (gamepad1.b) {
                    kickerMotor.setVelocity(kickerVelocity);
                } else {
                    kickerMotor.setPower(0);
                }
            }

            // Y Button: Launcher Motor
            if (launcherMotor != null) {
                if (gamepad1.y) {
                    launcherMotor.setVelocity(launcherVelocity);
                } else {
                    launcherMotor.setPower(0);
                }
            }

            // ================== TELEMETRY DISPLAY ==================
            telemetry.addData("Runtime", "%.2f", runtime.seconds());
            if (intakeMotor != null) {
                telemetry.addData("Intake (A)", "Target: %.0f | Actual: %.0f ticks/s",
                        intakeVelocity, intakeMotor.getVelocity());
            }
            if (kickerMotor != null) {
                telemetry.addData("Kicker (B)", "Target: %.0f | Actual: %.0f ticks/s",
                        kickerVelocity, kickerMotor.getVelocity());
            }
            if (launcherMotor != null) {
                telemetry.addData("Launcher (Y)", "Target: %.0f | Actual: %.0f ticks/s",
                        launcherVelocity, launcherMotor.getVelocity());
            }
            telemetry.update();
        }

        // Stop all motors on exit
        if (launcherMotor != null) launcherMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (kickerMotor != null) kickerMotor.setPower(0);
    }
}
