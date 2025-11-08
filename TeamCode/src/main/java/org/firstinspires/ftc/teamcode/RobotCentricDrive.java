package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple Robot-Centric Mecanum Drive
 * No IMU required - all movement relative to robot orientation
 * Perfect for beginners or when IMU is not available
 */
@TeleOp(name="Robot Centric Drive", group="Linear Opmode")
public class RobotCentricDrive extends LinearOpMode {

    // Declare OpMode members
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Drive control settings
    private double normalSpeed = 0.8;        // Normal driving speed
    private double precisionSpeed = 0.4;     // Precision mode speed
    
    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        
        telemetry.addData("Status", "Robot-Centric Drive Ready");
        telemetry.addData("Drive Type", "Robot-Centric (No IMU needed)");
        telemetry.addData("Motors", "312 RPM Precision Drive Motors");
        telemetry.addData("Controls", "Left stick=translate, Right stick=rotate");
        telemetry.addData("Modes", "Right Bumper=precision, Left Bumper=turbo");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // ================== SPEED MODE SELECTION ==================
            
            double currentMaxSpeed;
            if (gamepad1.right_bumper) {
                currentMaxSpeed = precisionSpeed;    // Precision mode
            } else if (gamepad1.left_bumper) {
                currentMaxSpeed = 1.0;               // Turbo mode
            } else {
                currentMaxSpeed = normalSpeed;       // Normal mode
            }
            
            // ================== ROBOT-CENTRIC DRIVETRAIN CONTROL ==================
            
            // Get joystick inputs
            double drive = -gamepad1.left_stick_y;    // Forward/backward
            double strafe = gamepad1.left_stick_x;     // Left/right
            double twist = gamepad1.right_stick_x;     // Rotation
            
            // Apply deadzone
            drive = Math.abs(drive) > 0.05 ? drive : 0;
            strafe = Math.abs(strafe) > 0.05 ? strafe : 0;
            twist = Math.abs(twist) > 0.05 ? twist : 0;
            
            // Calculate motor powers for mecanum drive (robot-centric)
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
            
            // Set motor powers
            robot.setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            
            // ================== MECHANISM CONTROLS ==================
            
            // Intake control - High speed motor
            if (gamepad1.y) {
                robot.setIntakePower(0.8);    // Moderate power for 6000+ RPM motor
            } else if (gamepad1.x) {
                robot.setIntakePower(-0.8);   // Moderate power for 6000+ RPM motor
            } else {
                robot.setIntakePower(0);
            }
            
            // ================== TELEMETRY ==================
            
            String speedMode = gamepad1.right_bumper ? "PRECISION" : 
                              gamepad1.left_bumper ? "TURBO" : "NORMAL";
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Mode", "ROBOT-CENTRIC");
            telemetry.addData("Speed Mode", speedMode + " (%.1f)", currentMaxSpeed);
            telemetry.addData("Input", "D:%.2f S:%.2f T:%.2f", drive, strafe, twist);
            telemetry.addData("Motors", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Mechanisms", "Intake:%.2f",
                            robot.intakeMotor.getPower());
            
            telemetry.update();
        }
    }
}
