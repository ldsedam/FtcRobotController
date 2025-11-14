package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Far Auto Red Mode", group = "Competition")
public class FarAutoRedMode extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;
    private static final int TARGET_APRILTAG_ID = 24;

    private enum AutoState {
        AIMING,
        SHOOTING,
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
        Limelight3A limelight = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -300.0;
        final double kickerLaunchTargetRPM = 300.0;
        final double farLauncherTargetRPM = 3200.0;

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerLaunchVelocity = (kickerLaunchTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double farLauncherVelocity = (farLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        // ===================== STATE AND CONTROL VARIABLES =====================
        AutoState currentState = AutoState.AIMING;
        final ElapsedTime shootTimer = new ElapsedTime();

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

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Limelight", "❌ Not found");
        }

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "BackRightDrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Far Auto Red Ready");
        telemetry.update();

        waitForStart();

        // ===================== MAIN AUTONOMOUS LOOP =====================
        while (opModeIsActive() && currentState != AutoState.DONE) {

            LLResult llResult = null;
            if (limelight != null) {
                llResult = limelight.getLatestResult();
            }

            double twist = 0;
            double intakeTargetVelocity = 0;
            double kickerTargetVelocity = 0;
            double launcherTargetVelocity = 0;

            boolean aprilTagFound = false;

            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult res : fiducialResults) {
                    if (res.getFiducialId() == TARGET_APRILTAG_ID) {
                        aprilTagFound = true;
                        double tx = res.getTargetXDegrees();
                        final double AIM_TOLERANCE = 1.5; // Degrees

                        if (currentState == AutoState.AIMING) {
                            double aimError = tx + 5.0; // Offset to aim 5 degrees to the right
                            if (Math.abs(aimError) > AIM_TOLERANCE) {
                                double turnKp = 0.05;
                                double minTurnPower = 0.5;
                                twist = (turnKp * aimError) + Math.copySign(minTurnPower, aimError);
                                twist = Math.max(-0.8, Math.min(0.8, twist));
                            } else {
                                twist = 0;
                                currentState = AutoState.SHOOTING;
                                shootTimer.reset();
                            }
                        }
                        break; // Exit loop once target tag is found
                    }
                }
            }

            switch (currentState) {
                case AIMING:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    break;

                case SHOOTING:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    if (launcherMotor != null && Math.abs(launcherMotor.getVelocity() - farLauncherVelocity) < (farLauncherVelocity * 0.10)) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                    if (shootTimer.seconds() > 5.0) {
                        currentState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    // Stop all motors
                    break;
            }

            leftFrontDrive.setPower(twist);
            rightFrontDrive.setPower(-twist);
            leftBackDrive.setPower(twist);
            rightBackDrive.setPower(-twist);

            if (launcherMotor != null) launcherMotor.setVelocity(launcherTargetVelocity);
            if (intakeMotor != null) intakeMotor.setVelocity(intakeTargetVelocity);
            if (kickerMotor != null) kickerMotor.setVelocity(kickerTargetVelocity);

            telemetry.addData("Mode", currentState);
            if(aprilTagFound){
                telemetry.addData("April Tag", "ID %d Visible", TARGET_APRILTAG_ID);
            } else {
                telemetry.addData("April Tag", "ID %d Not Visible", TARGET_APRILTAG_ID);
            }
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
        if (limelight != null) {
            limelight.stop();
        }
    }
}
