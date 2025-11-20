package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "automodered", group = "Competition")
public class automodered extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;
    private static final int TARGET_APRILTAG_ID = 24;

    private enum AutoState {
        SHOOTING,
        DELAY,
        TURN_LEFT,
        DRIVING_FORWARD,
        TURN_RIGHT,
        START_INTAKE,
        DRIVE_FORWARD_FINAL,
        BACKING_UP,
        TURN_LEFT_90,
        BACKUP_FINAL,
        TURN_RIGHT_10,
        AIM_FAR_LAUNCH,
        FAR_LAUNCH,
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
        NormalizedColorSensor colorSensor = null;
        Limelight3A limelight = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -300.0;
        final double kickerLaunchTargetRPM = 300.0;
        final double kickerIntakeTargetRPM = 200.0; // From MainTeleopOpMode
        final double farLauncherTargetRPM = 3200.0; // From MainTeleopOpMode

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerLaunchVelocity = (kickerLaunchTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerIntakeVelocity = (kickerIntakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double farLauncherVelocity = (farLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        final PIDFCoefficients pidf_6000_rpm = new PIDFCoefficients(0.6, 0.5, 0.0, 12.0);
        final PIDFCoefficients pidf_312_rpm = new PIDFCoefficients(9.0, 3.0, 0.0, 0.0);

        // ===================== STATE AND CONTROL VARIABLES =====================
        AutoState currentState = AutoState.SHOOTING;
        AutoState nextState = null;
        final ElapsedTime stateTimer = new ElapsedTime();

        // ===================== INITIALIZATION =====================
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_6000_rpm);
            telemetry.addData("Launcher Motor", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Launcher Motor", "❌ Not found");
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
            intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
            telemetry.addData("Intake Motor", "✅ Connected");
        } catch (Exception e) {
            telemetry.addData("Intake Motor", "❌ Not found");
        }

        try {
            kickerMotor = hardwareMap.get(DcMotorEx.class, "KickerMotor");
            kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            kickerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
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
            colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
            if (colorSensor instanceof SwitchableLight) {
                ((SwitchableLight) colorSensor).enableLight(true);
            }
            telemetry.addData("Color Sensor", "✅ Connected");
        } catch (Exception e) {
            colorSensor = null;
            telemetry.addData("Color Sensor", "❌ Not found");
        }
        
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
            telemetry.addData("Limelight", "✅ Connected");
        } catch (Exception e) {
            limelight = null;
            telemetry.addData("Limelight", "❌ Not found");
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

            boolean pixelDetected = false;
            if (colorSensor instanceof DistanceSensor) {
                double distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                if (distance < 5) {
                    pixelDetected = true;
                }
            }
            
            LLResult llResult = null;
            if (limelight != null) {
                llResult = limelight.getLatestResult();
            }

            double intakeTargetVelocity = 0;
            double kickerTargetVelocity = 0;
            double launcherTargetVelocity = 0;
            double leftPower = 0;
            double rightPower = 0;

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
                    if (stateTimer.seconds() > 3.0) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.TURN_LEFT;
                        stateTimer.reset();
                    }
                    break;

                case DELAY:
                    if (stateTimer.seconds() > 0.5) {
                        currentState = nextState;
                        stateTimer.reset();
                    }
                    break;

                case TURN_LEFT:
                    leftPower = -0.5;
                    rightPower = 0.5;
                    if (stateTimer.seconds() > 0.22) { // Adjust this value to get a 10 degree turn
                        currentState = AutoState.DELAY;
                        nextState = AutoState.DRIVING_FORWARD;
                        stateTimer.reset();
                    }
                    break;

                case DRIVING_FORWARD:
                    leftPower = 0.5;
                    rightPower = 0.5;
                    if (stateTimer.seconds() > 0.9) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.TURN_RIGHT;
                        stateTimer.reset();
                    }
                    break;

                case TURN_RIGHT:
                    leftPower = 0.5;
                    rightPower = -0.5;
                    if (stateTimer.seconds() > 0.80) { // Adjust this value for a 90 degree turn
                        currentState = AutoState.DELAY;
                        nextState = AutoState.START_INTAKE;
                        stateTimer.reset();
                    }
                    break;

                case START_INTAKE:
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    if (stateTimer.seconds() > 2.0) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.DRIVE_FORWARD_FINAL;
                        stateTimer.reset();
                    }
                    break;

                case DRIVE_FORWARD_FINAL:
                    leftPower = 0.5;
                    rightPower = 0.5;
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    if (stateTimer.seconds() > 2.0) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.BACKING_UP;
                        stateTimer.reset();
                    }
                    break;

                case BACKING_UP:
                    leftPower = -0.5;
                    rightPower = -0.5;
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    if (stateTimer.seconds() > 2.0) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.TURN_LEFT_90;
                        stateTimer.reset();
                    }
                    break;

                case TURN_LEFT_90:
                    leftPower = -0.5;
                    rightPower = 0.5;
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    if (stateTimer.seconds() > 0.85) { // Adjust this value for a 90 degree turn
                        currentState = AutoState.DELAY;
                        nextState = AutoState.BACKUP_FINAL;
                        stateTimer.reset();
                    }
                    break;

                case BACKUP_FINAL:
                    leftPower = -0.5;
                    rightPower = -0.5;
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    if (stateTimer.seconds() > 0.5) {
                        currentState = AutoState.DELAY;
                        nextState = AutoState.TURN_RIGHT_10;
                        stateTimer.reset();
                    }
                    break;

                case TURN_RIGHT_10:
                    leftPower = 0.5;
                    rightPower = -0.5;
                    if (stateTimer.seconds() > 0.22) { // Adjust this value for a 10 degree turn
                        currentState = AutoState.DELAY;
                        nextState = AutoState.AIM_FAR_LAUNCH;
                        stateTimer.reset();
                    }
                    break;

                case AIM_FAR_LAUNCH:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    boolean aprilTagFound = false;
                    if (llResult != null && llResult.isValid()) {
                        List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                        for (LLResultTypes.FiducialResult res : fiducialResults) {
                            if (res.getFiducialId() == TARGET_APRILTAG_ID) {
                                aprilTagFound = true;
                                double tx = res.getTargetXDegrees();
                                if (tx < -1.0 || tx > 7.0) {
                                    double turnKp = 0.05;
                                    double minTurnPower = 0.45;
                                    double aimError = tx;
                                    double twist = (turnKp * aimError) + Math.copySign(minTurnPower, aimError);
                                    twist = Math.max(-0.75, Math.min(0.75, twist));
                                    leftPower = twist;
                                    rightPower = -twist;
                                } else {
                                    currentState = AutoState.FAR_LAUNCH;
                                    stateTimer.reset();
                                }
                                break;
                            }
                        }
                    }
                    if (!aprilTagFound && stateTimer.seconds() > 3.0) {
                        currentState = AutoState.FAR_LAUNCH;
                        stateTimer.reset();
                    }
                    break;

                case FAR_LAUNCH:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    if (launcherMotor != null && Math.abs(launcherMotor.getVelocity() - farLauncherVelocity) < (farLauncherVelocity * 0.10)) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                    if (stateTimer.seconds() > 2.0) {
                        currentState = AutoState.DONE;
                    }
                    break;

                case DONE:
                    // Stop all motors
                    leftPower = 0;
                    rightPower = 0;
                    launcherTargetVelocity = 0;
                    intakeTargetVelocity = 0;
                    kickerTargetVelocity = 0;
                    break;
            }

            leftFrontDrive.setPower(leftPower);
            rightFrontDrive.setPower(rightPower);
            leftBackDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);

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
