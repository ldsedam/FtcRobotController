package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "Main TeleOp Mode Blue", group = "Competition")
public class MainTeleopOpModeBlue extends LinearOpMode {

    // Ticks per revolution for different goBILDA motors
    private static final double TICKS_PER_REV_6000_RPM = 28.0;
    private static final double TICKS_PER_REV_312_RPM = 537.6;

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

        // ===================== TARGET VELOCITIES =====================
        final double intakeTargetRPM = -300.0; // Target for 312 RPM motor
        final double kickerIntakeTargetRPM = 200.0;
        final double kickerLaunchTargetRPM = 300.0;
        final double closeLauncherTargetRPM = 2350.0;
        final double midLauncherTargetRPM = 3000.0;
        final double farLauncherTargetRPM = 3750.0;

        final double intakeVelocity = (intakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerIntakeVelocity = (kickerIntakeTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double kickerLaunchVelocity = (kickerLaunchTargetRPM / 60.0) * TICKS_PER_REV_312_RPM;
        final double closeLauncherVelocity = (closeLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;
        final double midLauncherVelocity = (midLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;
        final double farLauncherVelocity = (farLauncherTargetRPM / 60.0) * TICKS_PER_REV_6000_RPM;

        final PIDFCoefficients pidf_6000_rpm = new PIDFCoefficients(0.6, 0.5, 0.0, 12.0);
        final PIDFCoefficients pidf_312_rpm = new PIDFCoefficients(9.0, 3.0, 0.0, 0.0);

        // ===================== STATE AND CONTROL VARIABLES =====================
        boolean intakeSystemToggle = false;
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
                if (distance < 5) {
                    pixelDetected = true;
                }
            }

            // --- DRIVER INPUT ---
            boolean rbPressed = gamepad1.right_bumper;
            boolean aPressed = gamepad1.a;
            boolean bPressed = gamepad1.b;
            boolean xPressed = gamepad1.x;
            boolean yPressed = gamepad1.y;
            boolean startPressed = gamepad1.start;
            boolean backPressed = gamepad1.back;

            // --- STATE AND VELOCITY VARIABLES ---
            if (rbPressed && !lastRbPress) {
                intakeSystemToggle = !intakeSystemToggle;
            }
            lastRbPress = rbPressed;

            if (startPressed && !lastStartPress) {
                fieldRelative = !fieldRelative;
            }
            lastStartPress = startPressed;

            if (backPressed && !lastBackPress && imu != null) {
                imu.resetYaw();
            }
            lastBackPress = backPressed;

            double intakeTargetVelocity = 0;
            double kickerTargetVelocity = 0;
            double launcherTargetVelocity = 0;

            // --- DRIVETRAIN CONTROL ---
            double drive = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            drive = cubicScale(drive);
            strafe = cubicScale(strafe);
            twist = cubicScale(twist);

            if (fieldRelative && imu != null) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                double robotHeading = orientation.getYaw(AngleUnit.RADIANS);
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

            // --- MECHANISM CONTROL LOGIC (PRIORITY-BASED) ---
            if (aPressed) {
                intakeSystemToggle = false;
                launcherTargetVelocity = farLauncherVelocity;
                if (HoodServo != null) {
                    HoodServo.setPosition(0.30);
                }
                if (launcherMotor != null) {
                    if (Math.abs(launcherMotor.getVelocity() - farLauncherVelocity) < (farLauncherVelocity * 0.10)) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                }
            } else if (bPressed) {
                intakeSystemToggle = false;
                launcherTargetVelocity = midLauncherVelocity;
                if (HoodServo != null) {
                    HoodServo.setPosition(0.45);
                }
                if (launcherMotor != null) {
                    if (Math.abs(launcherMotor.getVelocity() - midLauncherVelocity) < (midLauncherVelocity * 0.10)) {
                        intakeTargetVelocity = intakeVelocity;
                        kickerTargetVelocity = kickerLaunchVelocity;
                    }
                }
            } else if (yPressed) {
                intakeSystemToggle = false;
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
            } else if (intakeSystemToggle) {
                intakeTargetVelocity = intakeVelocity;
                if (!pixelDetected) {
                    kickerTargetVelocity = kickerIntakeVelocity;
                }
            } else {
                if (xPressed) {
                    intakeTargetVelocity = -intakeVelocity;
                }
            }
            lastYPress = yPressed;

            // --- MOTOR AND SERVO OUTPUT ---
            leftFrontDrive.setPower(lfPower);
            rightFrontDrive.setPower(rfPower);
            leftBackDrive.setPower(lbPower);
            rightBackDrive.setPower(rbPower);

            if (launcherMotor != null) launcherMotor.setVelocity(launcherTargetVelocity);
            if (intakeMotor != null) intakeMotor.setVelocity(intakeTargetVelocity);
            if (kickerMotor != null) kickerMotor.setVelocity(kickerTargetVelocity);

            // ================== TELEMETRY DISPLAY ==================
            telemetry.addData("Mode", intakeSystemToggle ? "Intake System ON" : "Launch/Individual");
            telemetry.addData("Drive Mode", fieldRelative ? "FIELD CENTRIC" : "ROBOT CENTRIC");
            telemetry.addData("Object Detected", pixelDetected ? "Yes" : "No");
            telemetry.addData("Distance (cm)", "%.2f", distance);
            if (HoodServo != null) {
                telemetry.addData("Hood Servo", "%.2f", HoodServo.getPosition());
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
