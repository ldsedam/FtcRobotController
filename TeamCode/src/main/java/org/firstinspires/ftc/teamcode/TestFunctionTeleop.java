package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Advanced Mecanum Drive for precision goBILDA motors
 * Optimized for 312 RPM drive motors with excellent control
 * Features field-relative driving option
 */
@TeleOp(name="TestFunctionTeleop", group="Linear Opmode")
public class TestFunctionTeleop extends LinearOpMode {

    // Declare OpMode members
    private final RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    // Drive control settings
    private static final double MAX_SPEED = 1.0;           // Maximum drive speed (312 RPM can handle full power)
    private static final double NORMAL_SPEED = 0.8;        // Normal driving speed
    private static final double PRECISION_SPEED = 0.4;     // Precision mode speed

    // Acceleration limiting (less critical with 312 RPM motors but still useful)
    private static final double ACCELERATION_LIMIT = 0.15; // Can be higher with precision motors
    private double lastLeftFrontPower = 0;
    private double lastRightFrontPower = 0;
    private double lastLeftBackPower = 0;
    private double lastRightBackPower = 0;

    // Control mode
    private boolean fieldRelative = false;
    private boolean lastModeButton = false;
    private boolean lastResetButton = false;

    @Override
    public void runOpMode() {

        double currentMaxSpeed = NORMAL_SPEED;

        // Initialize the hardware variables
        robot.init(hardwareMap);

        // Check if IMU was initialized successfully
        if (robot.imu == null) {
            fieldRelative = false; // Force robot-centric if no IMU
            telemetry.addData("IMU Status", "Not found - Robot-centric only");
        } else {
            telemetry.addData("IMU Status", "Connected - Field-centric available");
        }

        telemetry.addData("Status", "Advanced Mecanum Drive Ready");
        telemetry.addData("Motors", "312 RPM Precision Drive Motors");
        telemetry.addData("Features", "Precise control, encoder feedback");
        telemetry.addData("Controls", "Left stick=translate, Right stick=rotate");
        telemetry.addData("Modes", "RB=precision, LB=turbo, START=field-centric, BACK=reset heading");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ================== SPEED MODE SELECTION ==================

            if (gamepad1.right_bumper) {
                currentMaxSpeed = PRECISION_SPEED;    // Precision mode
            } else if (gamepad1.left_bumper) {
                currentMaxSpeed = MAX_SPEED;          // Turbo mode
            } else {
                currentMaxSpeed = NORMAL_SPEED;       // Normal mode
            }

            // ================== FIELD RELATIVE TOGGLE ==================

            boolean currentModeButton = gamepad1.start;
            if (currentModeButton && !lastModeButton && robot.imu != null) {
                fieldRelative = !fieldRelative;
            }
            lastModeButton = currentModeButton;

            // ================== IMU HEADING RESET ==================

            boolean currentResetButton = gamepad1.back;
            if (currentResetButton && !lastResetButton && robot.imu != null) {
                robot.imu.resetYaw();
            }
            lastResetButton = currentResetButton;

            // ================== DRIVETRAIN CONTROL ==================

            // Get raw joystick inputs
            double rawDrive = -gamepad1.left_stick_y;    // Forward/backward
            double rawStrafe = gamepad1.left_stick_x;     // Left/right
            double rawTwist = gamepad1.right_stick_x;     // Rotation

            // Apply deadzone
            double drive = Math.abs(rawDrive) > 0.05 ? rawDrive : 0;
            double strafe = Math.abs(rawStrafe) > 0.05 ? rawStrafe : 0;
            double twist = Math.abs(rawTwist) > 0.05 ? rawTwist : 0;

            // Apply cubic scaling for better control (less critical with 312 RPM but still beneficial)
            drive = cubicScale(drive);
            strafe = cubicScale(strafe);
            twist = cubicScale(twist) * 0.9;  // Slightly less rotation scaling needed

            // ================== FIELD RELATIVE CALCULATION ==================

            if (fieldRelative && robot.imu != null) {
                // Get robot heading
                double robotHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                // Rotate the drive and strafe values based on robot heading
                double rotX = drive * Math.cos(-robotHeading) - strafe * Math.sin(-robotHeading);
                double rotY = drive * Math.sin(-robotHeading) + strafe * Math.cos(-robotHeading);

                drive = rotX;
                strafe = rotY;
                // Note: twist (rotation) stays the same in field-centric
            }

            // Calculate motor powers for mecanum drive
            double leftFrontPower = drive + strafe + twist;
            double rightFrontPower = drive - strafe - twist;
            double leftBackPower = drive - strafe + twist;
            double rightBackPower = drive + strafe - twist;

            // Normalize wheel powers to prevent any from exceeding 1.0
            double maxPower = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
            );

            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;
            }

            // Apply speed scaling
            leftFrontPower *= currentMaxSpeed;
            rightFrontPower *= currentMaxSpeed;
            leftBackPower *= currentMaxSpeed;
            rightBackPower *= currentMaxSpeed;

            // Apply acceleration limiting to smooth movements (less critical with 312 RPM motors)
            leftFrontPower = applyAccelLimit(leftFrontPower, lastLeftFrontPower);
            rightFrontPower = applyAccelLimit(rightFrontPower, lastRightFrontPower);
            leftBackPower = applyAccelLimit(leftBackPower, lastLeftBackPower);
            rightBackPower = applyAccelLimit(rightBackPower, lastRightBackPower);

            // Set motor powers
            robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Store powers for next acceleration limit calculation
            lastLeftFrontPower = leftFrontPower;
            lastRightFrontPower = rightFrontPower;
            lastLeftBackPower = leftBackPower;
            lastRightBackPower = rightBackPower;

            // ================== MECHANISM CONTROLS ==================

            if (gamepad1.a) {
                if (robot.HoodServo != null) {
                    robot.HoodServo.setPosition(0.0);
                }
            } else if (gamepad1.y) {
                if (robot.HoodServo != null) {
                    robot.HoodServo.setPosition(0.65);
                }
                robot.setIntakePower(0.8);    // Moderate power for 6000+ RPM motor
            } else if (gamepad1.x) {
                robot.setIntakePower(-0.8);   // Moderate power for 6000+ RPM motor
            } else {
                robot.setIntakePower(0);
            }

            // ================== TELEMETRY ==================

            String speedMode = gamepad1.right_bumper ? "PRECISION" :
                              gamepad1.left_bumper ? "TURBO" : "NORMAL";

            String driveMode = "ROBOT CENTRIC";
            if (robot.imu == null) {
                driveMode = "ROBOT CENTRIC (No IMU)";
            } else if (fieldRelative) {
                driveMode = "FIELD CENTRIC";
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Speed Mode", speedMode + " (%.1f)", currentMaxSpeed);
            telemetry.addData("Drive Mode", driveMode);
            if (robot.imu != null) {
                telemetry.addData("Robot Heading", "%.1f°",
                                robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Controls", "START=toggle field/robot, BACK=reset heading");
            }
            telemetry.addData("Input", "D:%.2f S:%.2f T:%.2f", drive, strafe, twist);
            telemetry.addData("Motors", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f",
                            leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Mechanisms", "Intake:%.2f",
                            robot.intakeMotor.getPower());
            if (robot.HoodServo != null) {
                telemetry.addData("Hood Servo Position", "%.2f", robot.HoodServo.getPosition());
            }

            telemetry.update();
        }
    }

    /**
     * Apply cubic scaling for smoother control
     * Provides more precise control at low speeds while maintaining full power at high speeds
     */
    private double cubicScale(double input) {
        return input * input * input * 0.7 + input * 0.3;
    }

    /**
     * Apply acceleration limiting for smooth control
     * Less critical with 312 RPM motors but still provides smoother operation
     */
    private double applyAccelLimit(double targetPower, double currentPower) {
        double powerChange = targetPower - currentPower;

        if (Math.abs(powerChange) > ACCELERATION_LIMIT) {
            if (powerChange > 0) {
                return currentPower + ACCELERATION_LIMIT;
            } else {
                return currentPower - ACCELERATION_LIMIT;
            }
        }

        return targetPower;
    }
}
