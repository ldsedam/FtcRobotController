package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PID Motor Test", group = "Testing")
public class PIDMotorTest extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;

    @Override
    public void runOpMode() {

        final ElapsedTime runtime = new ElapsedTime();

        // ===================== MOTOR DECLARATIONS =====================
        DcMotorEx launcherMotor = null;
        DcMotorEx intakeMotor = null;
        DcMotorEx kickerMotor = null;
        NormalizedColorSensor colorSensor = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -250.0; // Target for 312 RPM motor
        final double kickerTargetRPM = 200.0;
        final double launcherTargetRPM = 2000.0;

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerVelocity = (kickerTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double launcherVelocity = (launcherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        final PIDFCoefficients pidf_6000_rpm = new PIDFCoefficients(8.0, 3.0, 0.0, 12.0);
        final PIDFCoefficients pidf_312_rpm = new PIDFCoefficients(9.0, 3.0, 0.0, 0.0);

        // ===================== STATE VARIABLES =====================
        boolean intakeSystemToggle = false;
        boolean lastRbPress = false;

        // ===================== INITIALIZATION =====================
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_6000_rpm);
            telemetry.addData("Launcher Motor (6000 RPM)", "✅ Connected");
        } catch (Exception e) {
            launcherMotor = null;
            telemetry.addData("Launcher Motor (6000 RPM)", "❌ Not found");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
            telemetry.addData("Intake Motor (312 RPM)", "✅ Connected");
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addData("Intake Motor (312 RPM)", "❌ Not found");
        }

        try {
            kickerMotor = hardwareMap.get(DcMotorEx.class, "KickerMotor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
            telemetry.addData("Kicker Motor (312 RPM)", "✅ Connected");
        } catch (Exception e) {
            kickerMotor = null;
            telemetry.addData("Kicker Motor (312 RPM)", "❌ Not found");
        }

        try {
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
            telemetry.addData("Color Sensor", "✅ Connected");
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("Color Sensor", "❌ Not found");
        }

        telemetry.addData("Status", "PID Motor Test Ready");
        telemetry.addData("-", "----------------------------------");
        telemetry.addData("RB Toggle", "Intake + Kicker System");
        telemetry.addData("A/B/Y", "Individual Motor Tests");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ===================== MAIN CONTROL LOOP =====================
        while (opModeIsActive()) {

            // --- Distance Sensor Logic ---
            boolean pixelDetected = false;
            double distance = -1.0;
            if (colorSensor instanceof DistanceSensor) {
                distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                if (distance < 5) {
                    pixelDetected = true;
                }
            }

            // --- Intake System Toggle Logic ---
            boolean currentRbPress = gamepad1.right_bumper;
            if (currentRbPress && !lastRbPress) {
                intakeSystemToggle = !intakeSystemToggle;
            }
            lastRbPress = currentRbPress;

            if (intakeSystemToggle) {
                // --- INTAKE SYSTEM IS ON ---
                if (intakeMotor != null) {
                    intakeMotor.setVelocity(intakeVelocity);
                }

                if (kickerMotor != null) {
                    if (pixelDetected) {
                        kickerMotor.setPower(0); // Stop kicker if pixel is detected
                    } else {
                        kickerMotor.setVelocity(kickerVelocity);
                    }
                }

                // Turn off launcher if Y is not held in this mode
                if (launcherMotor != null && !gamepad1.y) {
                    launcherMotor.setPower(0);
                }

            } else {
                // --- INDIVIDUAL MOTOR CONTROL MODE ---
                // A Button: Intake Motor
                if (intakeMotor != null) {
                    if (gamepad1.a) {
                        intakeMotor.setVelocity(intakeVelocity);
                    } else if (gamepad1.x) {
                        intakeMotor.setVelocity(-intakeVelocity);
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
            }

            // Y Button: Launcher Motor (works in both modes)
            if (launcherMotor != null) {
                if (gamepad1.y) {
                    launcherMotor.setVelocity(launcherVelocity);
                } else if (!intakeSystemToggle) { // Only turn off if not in toggle mode
                    launcherMotor.setPower(0);
                }
            }

            // ================== TELEMETRY DISPLAY ==================
            telemetry.addData("Runtime", "%.2f", runtime.seconds());
            telemetry.addData("Mode", intakeSystemToggle ? "Intake System ON" : "Individual Control");
            telemetry.addData("Object Detected", pixelDetected);
            telemetry.addData("Distance (cm)", "%.2f", distance);
            if (intakeMotor != null) {
                telemetry.addData("Intake", "Target: %.0f | Actual: %.0f t/s",
                        intakeVelocity, intakeMotor.getVelocity());
            }
            if (kickerMotor != null) {
                telemetry.addData("Kicker", "Target: %.0f | Actual: %.0f t/s",
                        kickerVelocity, kickerMotor.getVelocity());
            }
            if (launcherMotor != null) {
                telemetry.addData("Launcher", "Target: %.0f | Actual: %.0f t/s",
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
