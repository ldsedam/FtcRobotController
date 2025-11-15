package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Auto Mode", group = "Competition")
public class BasicAutoMode extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;

    private enum AutoState {
        SHOOTING,
        DRIVING_FORWARD,
        DONE
    }

    @Override
    public void runOpMode() {

        // ===================== HARDWARE DECLARATIONS =====================
        DcMotorEx launcherMotor = null;
        DcMotorEx intakeMotor = null;
        DcMotorEx kickerMotor = null;
        DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
        Servo HoodServo = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -300.0;
        final double kickerLaunchTargetRPM = 300.0;
        final double farLauncherTargetRPM = 3200.0;

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerLaunchVelocity = (kickerLaunchTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double farLauncherVelocity = (farLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        // ===================== STATE AND CONTROL VARIABLES =====================
        AutoState currentState = AutoState.SHOOTING;
        final ElapsedTime stateTimer = new ElapsedTime();

        // ===================== INITIALIZATION =====================
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Launcher Motor", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Launcher Motor", "❌ Not found");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Intake Motor", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Intake Motor", "❌ Not found");
        }

        try {
            kickerMotor = hardwareMap.get(DcMotorEx.class, "KickerMotor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Kicker Motor", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Kicker Motor", "❌ Not found");
        }

        try {
            HoodServo = hardwareMap.get(Servo.class, "HoodServo");
            telemetry.addData("Hood Servo", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Hood Servo", "❌ Not found");
        }

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Basic Auto Mode Ready");
        telemetry.update();

        waitForStart();
        stateTimer.reset();

        // ===================== MAIN AUTONOMOUS LOOP =====================
        while (opModeIsActive() && currentState != AutoState.DONE) {

            double intakeTargetVelocity = 0;
            double kickerTargetVelocity = 0;
            double launcherTargetVelocity = 0;
            double drivePower = 0;

            switch (currentState) {
                case SHOOTING:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    if (launcherMotor != null && Math.abs(launcherMotor.getVelocity() - farLauncherVelocity) < (farLauncherVelocity * 0.10)) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                    if (stateTimer.seconds() > 5.0) {
                        currentState = AutoState.DRIVING_FORWARD;
                        stateTimer.reset();
                    }
                    break;

                case DRIVING_FORWARD:
                    drivePower = 0.5;
                    if (stateTimer.seconds() > 3.0) {
                        currentState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    // Stop all motors
                    drivePower = 0;
                    launcherTargetVelocity = 0;
                    intakeTargetVelocity = 0;
                    kickerTargetVelocity = 0;
                    break;
            }

            leftFrontDrive.setPower(drivePower);
            rightFrontDrive.setPower(drivePower);
            leftBackDrive.setPower(drivePower);
            rightBackDrive.setPower(drivePower);

            if (launcherMotor != null) launcherMotor.setVelocity(launcherTargetVelocity);
            if (intakeMotor != null) intakeMotor.setVelocity(intakeTargetVelocity);
            if (kickerMotor != null) kickerMotor.setVelocity(kickerTargetVelocity);

            telemetry.addData("Mode", currentState);
            telemetry.update();
        }

        // Stop all motors on exit
        if (launcherMotor != null) launcherMotor.setPower(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (kickerMotor != null) kickerMotor.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
