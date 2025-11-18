package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Main TeleOp Mode Blue", group = "Competition")
public class MainTeleopOpModeBlue extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;
    private static final int TARGET_APRILTAG_ID = 20;

    private enum RobotState {
        IDLE,
        TURNING_TO_SHOOT_FAR,
        SHOOTING_MID,
        SHOOTING_FAR,
        SHOOTING_CLOSE,
        INTAKE
    }

    @Override
    public void runOpMode() {

        final ElapsedTime runtime = new ElapsedTime();

        // ===================== HARDWARE DECLARATIONS =====================
        DcMotorEx launcherMotor = null;
        DcMotorEx intakeMotor = null;
        DcMotorEx kickerMotor = null;
        DcMotorEx leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive = null;
        NormalizedColorSensor colorSensor = null;
        Servo HoodServo = null;
        IMU imu = null;
        Limelight3A limelight = null;

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -300.0; // Target for 312 RPM motor
        final double kickerIntakeTargetRPM = 200.0;
        final double kickerLaunchTargetRPM = 300.0;
        final double closeLauncherTargetRPM = 2350.0;
        final double midLauncherTargetRPM = 2600.0;
        final double farLauncherTargetRPM = 3350.0;
        final double idleLauncherTargetRPM = 1000.0;

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerIntakeVelocity = (kickerIntakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerLaunchVelocity = (kickerLaunchTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double closeLauncherVelocity = (closeLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;
        final double midLauncherVelocity = (midLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;
        final double farLauncherVelocity = (farLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;
        final double idleLauncherVelocity = (idleLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        // final PIDFCoefficients pidf_6000_rpm = new PIDFCoefficients(0.6, 0.5, 0.0, 12.0);
        // final PIDFCoefficients pidf_312_rpm = new PIDFCoefficients(9.0, 3.0, 0.0, 0.0);

        // ===================== STATE AND CONTROL VARIABLES =====================
        boolean lastRbPress = false;
        boolean fieldRelative = false;
        boolean lastStartPress = false;
        boolean lastBackPress = false;
        boolean lastYPress = false;
        final ElapsedTime closeLaunchTimer = new ElapsedTime();

        // Drive control settings
        final double NORMAL_SPEED = 0.8;
        final double PRECISION_SPEED = 0.4;
        final double ACCELERATION_LIMIT = 0.15;

        // Acceleration limiting variables
        double lastLeftFrontPower = 0, lastRightFrontPower = 0, lastLeftBackPower = 0, lastRightBackPower = 0;
        RobotState currentState = RobotState.IDLE;


        // ===================== INITIALIZATION =====================
        try {
            launcherMotor = hardwareMap.get(DcMotorEx.class, "LauncherMotor");
            launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            // launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_6000_rpm);
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
            // intakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
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
            // kickerMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf_312_rpm);
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

        try {
            HoodServo = hardwareMap.get(Servo.class, "HoodServo");
            telemetry.addData("Hood Servo", "✅ Connected");
        } catch (Exception e) {
            HoodServo = null;
            telemetry.addData("Hood Servo", "❌ Not found");
        }

        try {
            imu = hardwareMap.get(IMU.class, "IMU");
            telemetry.addData("IMU", "✅ Connected");
        } catch (Exception e) {
            imu = null;
            telemetry.addData("IMU", "❌ Not found");
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

        telemetry.addData("Status", "Main TeleOp Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ===================== MAIN CONTROL LOOP =====================
        while (opModeIsActive()) {

            // --- SENSOR LOGIC ---
            boolean pixelDetected = false;
            double distance = -1.0;
            if (colorSensor instanceof DistanceSensor) {
                distance = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
                if (distance < 5.4) {
                    pixelDetected = true;
                }
            }

            LLResult llResult = null;
            if (limelight != null) {
                llResult = limelight.getLatestResult();
            }

            // --- DRIVER INPUT ---
            boolean rbPressed = gamepad1.right_bumper;
            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;
            boolean xPressed = gamepad1.x;
            boolean yPressed = gamepad1.y;
            boolean startPressed = gamepad1.start;
            boolean backPressed = gamepad1.back;

            // --- STATE TRANSITIONS ---
            // Handle button releases to cancel actions
            if (!aPressed && (currentState == RobotState.TURNING_TO_SHOOT_FAR || currentState == RobotState.SHOOTING_FAR)) {
                currentState = RobotState.IDLE;
            }
            if (!bPressed && currentState == RobotState.SHOOTING_MID) {
                currentState = RobotState.IDLE;
            }
            if (!yPressed && currentState == RobotState.SHOOTING_CLOSE) {
                currentState = RobotState.IDLE;
            }

            // Handle button presses to start actions (if IDLE)
            if (currentState == RobotState.IDLE) {
                if (aPressed) {
                    currentState = RobotState.TURNING_TO_SHOOT_FAR;
                } else if (bPressed) {
                    currentState = RobotState.SHOOTING_MID;
                } else if (yPressed) {
                    currentState = RobotState.SHOOTING_CLOSE;
                } else if (rbPressed && !lastRbPress) {
                    currentState = RobotState.INTAKE;
                }
            } else if (currentState == RobotState.INTAKE && rbPressed && !lastRbPress) {
                currentState = RobotState.IDLE;
            }

            // Field relative and IMU reset toggles are independent
            if (startPressed && !lastStartPress) {
                fieldRelative = !fieldRelative;
            }
            if (backPressed && !lastBackPress && imu != null) {
                imu.resetYaw();
            }

            lastRbPress = rbPressed;
            lastStartPress = startPressed;
            lastBackPress = backPressed;


            double intakeTargetVelocity = 0;
            double kickerTargetVelocity = 0;
            double launcherTargetVelocity = 0;
            double twist = 0;

            double robotHeading = 0;
            if (imu != null) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                robotHeading = orientation.getYaw(AngleUnit.RADIANS);
            }

            boolean aprilTagFound = false;

            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                for (LLResultTypes.FiducialResult res : fiducialResults) {
                    if (res.getFiducialId() == TARGET_APRILTAG_ID) {
                        aprilTagFound = true;
                        double tx = res.getTargetXDegrees();

                        if (currentState == RobotState.TURNING_TO_SHOOT_FAR) {
                            double aimError = tx - 1.0; // Target the middle of the window
                            // Fire when tx is between -3 and +5 degrees
                            if (tx < -3.0 || tx > 5.0) {
                                double turnKp = 0.05;
                                double minTurnPower = 0.45;
                                twist = (turnKp * aimError) + Math.copySign(minTurnPower, aimError);
                                // Clamp twist to a reasonable range
                                twist = Math.max(-0.75, Math.min(0.75, twist));
                            } else {
                                twist = 0;
                                currentState = RobotState.SHOOTING_FAR;
                            }
                        }
                        break; // Exit loop once target tag is found
                    }
                }
            }


            switch (currentState) {
                case TURNING_TO_SHOOT_FAR:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    // Turning logic is handled above
                    break;

                case SHOOTING_FAR:
                    launcherTargetVelocity = farLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.30);
                    }
                    if (launcherMotor != null) {
                        if (Math.abs(launcherMotor.getVelocity() - farLauncherVelocity) < (farLauncherVelocity * 0.12)) {
                            intakeTargetVelocity = intakeVelocity;
                            kickerTargetVelocity = kickerLaunchVelocity;
                        }
                    }
                    break;

                case SHOOTING_MID:
                    launcherTargetVelocity = midLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.45);
                    }
                    if (launcherMotor != null) {
                        if (Math.abs(launcherMotor.getVelocity() - midLauncherVelocity) < (midLauncherVelocity * 0.12)) {
                            intakeTargetVelocity = intakeVelocity;
                            kickerTargetVelocity = kickerLaunchVelocity;
                        }
                    }
                    break;

                case SHOOTING_CLOSE:
                    launcherTargetVelocity = closeLauncherVelocity;
                    if (HoodServo != null) {
                        HoodServo.setPosition(0.9);
                    }
                    if (!lastYPress) { // If the button was just pressed
                        closeLaunchTimer.reset();
                    }
                    if (closeLaunchTimer.seconds() >= 1.5) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                    break;

                case INTAKE:
                    launcherTargetVelocity = idleLauncherVelocity;
                    intakeTargetVelocity = intakeVelocity;
                    if (!pixelDetected) {
                        kickerTargetVelocity = kickerIntakeVelocity;
                    }
                    break;

                case IDLE:
                    launcherTargetVelocity = idleLauncherVelocity;
                    if (xPressed) {
                        intakeTargetVelocity = -intakeVelocity;
                        kickerTargetVelocity = -kickerLaunchVelocity;
                        launcherTargetVelocity = -farLauncherVelocity;
                    }
                    break;
            }
            lastYPress = yPressed;


            // --- DRIVETRAIN CONTROL ---
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            if (twist == 0) {
                twist = gamepad1.right_stick_x;
            }

            drive = cubicScale(drive);
            strafe = cubicScale(strafe);
            twist = cubicScale(twist);

            if (fieldRelative && imu != null) {
                double rotX = drive * Math.cos(-robotHeading) - strafe * Math.sin(-robotHeading);
                double rotY = drive * Math.sin(-robotHeading) + strafe * Math.cos(-robotHeading);
                drive = rotX;
                strafe = rotY;
            }

            double currentDriveSpeed = gamepad1.left_bumper ? PRECISION_SPEED : NORMAL_SPEED;

            double lfPower = (drive + strafe + twist) * currentDriveSpeed;
            double rfPower = (drive - strafe - twist) * currentDriveSpeed;
            double lbPower = (drive - strafe + twist) * currentDriveSpeed;
            double rbPower = (drive + strafe - twist) * currentDriveSpeed;

            // Normalize powers
            double maxPower = Math.max(1.0, Math.max(Math.abs(lfPower), Math.abs(rfPower)));
            maxPower = Math.max(maxPower, Math.max(Math.abs(lbPower), Math.abs(rbPower)));

            lfPower /= maxPower;
            rfPower /= maxPower;
            lbPower /= maxPower;
            rbPower /= maxPower;

            // Acceleration Limiting
            lfPower = applyAccelLimit(lfPower, lastLeftFrontPower, ACCELERATION_LIMIT);
            rfPower = applyAccelLimit(rfPower, lastRightFrontPower, ACCELERATION_LIMIT);
            lbPower = applyAccelLimit(lbPower, lastLeftBackPower, ACCELERATION_LIMIT);
            rbPower = applyAccelLimit(rbPower, lastRightBackPower, ACCELERATION_LIMIT);

            lastLeftFrontPower = lfPower;
            lastRightFrontPower = rfPower;
            lastLeftBackPower = lbPower;
            lastRightBackPower = rbPower;

            // --- MOTOR AND SERVO OUTPUT ---
            leftFrontDrive.setPower(lfPower);
            rightFrontDrive.setPower(rfPower);
            leftBackDrive.setPower(lbPower);
            rightBackDrive.setPower(rbPower);

            if (launcherMotor != null) launcherMotor.setVelocity(launcherTargetVelocity);
            if (intakeMotor != null) intakeMotor.setVelocity(intakeTargetVelocity);
            if (kickerMotor != null) kickerMotor.setVelocity(kickerTargetVelocity);

            // ================== TELEMETRY DISPLAY ==================
            telemetry.addData("Mode", currentState);
            telemetry.addData("Drive Mode", fieldRelative ? "FIELD CENTRIC" : "ROBOT CENTRIC");
            telemetry.addData("Object Detected", pixelDetected ? "Yes" : "No");
            telemetry.addData("Distance (cm)", "%.2f", distance);
            if(aprilTagFound){
                telemetry.addData("April Tag", "ID %d Visible", TARGET_APRILTAG_ID);
            } else {
                telemetry.addData("April Tag", "ID %d Not Visible", TARGET_APRILTAG_ID);
            }
            if (HoodServo != null) {
                telemetry.addData("Hood Servo", "%.2f", HoodServo.getPosition());
            }
            if (limelight != null && llResult != null && llResult.isValid()) {
                telemetry.addData("Limelight Target", "Visible");
                telemetry.addData("Limelight tx", "%.2f", llResult.getTx());
                telemetry.addData("Limelight ty", "%.2f", llResult.getTy());

                List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
                if (fiducialResults.size() > 0) {
                    for (LLResultTypes.FiducialResult res : fiducialResults) {
                        telemetry.addData("April Tag ID", res.getFiducialId());
                    }
                } else {
                    telemetry.addData("April Tag ID", "Not Visible");
                }

            } else {
                telemetry.addData("Limelight Target", "Not Visible");
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

    private double cubicScale(double input) {
        return input * input * input;
    }

    private double applyAccelLimit(double targetPower, double currentPower, double limit) {
        double powerChange = targetPower - currentPower;
        if (Math.abs(powerChange) > limit) {
            return currentPower + (powerChange > 0 ? limit : -limit);
        }
        return targetPower;
    }

}
