package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * TeleOp mode optimized for Logitech controller with goBILDA motors
 * - Low RPM motors (312 RPM) for precise drivetrain control
 * - High RPM motors (6000+ RPM) for fast mechanisms
 * - Comprehensive control mapping for Logitech F310/F710 controllers
 */
@TeleOp(name="Robot TeleOp - Logitech", group="Linear Opmode")
public class RobotTeleOpLogitech extends LinearOpMode {

    // Declare OpMode members
    private RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    
    // Control variables
    private double driveSpeed = 1.0;        // Normal drive speed (312 RPM motors can handle full power)
    private double precisionSpeed = 0.4;    // Precision drive speed
    private double intakeSpeed = 0.8;       // Intake speed (6000+ RPM motor)
    
    @Override
    public void runOpMode() {
        
        // Initialize the hardware variables
        robot.init(hardwareMap);
        
        // Send telemetry message to signify robot waiting
        telemetry.addData("Status", "Robot Ready - Logitech Controller");
        telemetry.addData("Drive Speed", "%.1f (Right Bumper = Precision)", driveSpeed);
        telemetry.addData("Drive Motors", "312 RPM (Precision Control)");
        telemetry.addData("Mechanism Motors", "6000+ RPM (High Speed)");
        telemetry.addData("Controls", "Left Stick = Drive, Right Stick = Turn/Strafe");
        telemetry.addData("Gamepad1", "Y=Intake, X=Outtake");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // ================== DRIVETRAIN CONTROL ==================
            
            // Get gamepad inputs
            double drive = -gamepad1.left_stick_y;    // Forward/backward (inverted for intuitive control)
            double strafe = gamepad1.left_stick_x;    // Left/right strafe
            double twist = gamepad1.right_stick_x;    // Rotation
            
            // Apply deadzone to prevent drift
            drive = Math.abs(drive) > 0.1 ? drive : 0;
            strafe = Math.abs(strafe) > 0.1 ? strafe : 0;
            twist = Math.abs(twist) > 0.1 ? twist : 0;
            
            // Determine speed mode
            double currentDriveSpeed = gamepad1.right_bumper ? precisionSpeed : driveSpeed;
            
            // Apply mecanum drive
            robot.mecanumDrive(drive, strafe, twist, currentDriveSpeed);
            
            // ================== INTAKE CONTROL ==================
            
            double intake = 0;
            if (gamepad1.y) {
                intake = intakeSpeed;       // Intake in
            } else if (gamepad1.x) {
                intake = -intakeSpeed;      // Intake out (outtake)
            }
            robot.setIntakePower(intake);
            
            // ================== TELEMETRY ==================
            
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Mode", gamepad1.right_bumper ? "PRECISION" : "NORMAL");
            telemetry.addData("Motors", "LF:%.2f RF:%.2f LB:%.2f RB:%.2f", 
                            robot.leftFrontDrive.getPower(),
                            robot.rightFrontDrive.getPower(), 
                            robot.leftBackDrive.getPower(),
                            robot.rightBackDrive.getPower());
            telemetry.addData("Intake Power", "%.2f", robot.intakeMotor.getPower());
            
            // Show encoder values for debugging
            int[] encoders = robot.getDriveEncoders();
            telemetry.addData("Encoders", "LF:%d RF:%d LB:%d RB:%d", 
                            encoders[0], encoders[1], encoders[2], encoders[3]);
            
            telemetry.update();
        }
    }
}
