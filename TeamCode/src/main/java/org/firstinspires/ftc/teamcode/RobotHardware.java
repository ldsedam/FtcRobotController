package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Hardware configuration for robot with goBILDA 5203 series motors
 * - Low RPM motors (312 RPM) for precise drivetrain control
 * - High RPM motors (6000+ RPM) for fast mechanisms
 * - Logitech controller support
 */
public class RobotHardware {
    
    /* Public OpMode members. */
    public DcMotor leftFrontDrive   = null;
    public DcMotor rightFrontDrive  = null;
    public DcMotor leftBackDrive    = null;
    public DcMotor rightBackDrive   = null;
    
    // High speed motors (6000+ RPM goBILDA motors)
    public DcMotor intakeMotor      = null;
    
    // PID controlled motors
    public DcMotor launcherMotor    = null;  // 5203 series 6000rpm for launcher (4500 RPM target)
    public DcMotor kickerMotor      = null;  // 5203 series 312rpm for kicker tasks (150 RPM target)
    
    // Servos
    public Servo HoodServo          = null;
    
    // IMU for field-centric drive (optional)
    public IMU imu                  = null;
    
    /* Local OpMode members. */
    HardwareMap hwMap               = null;
    
    /* Constructor */
    public RobotHardware() {
    }
    
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        
        // Define and Initialize Motors (note: need to use the names in your robot configuration)
        leftFrontDrive  = hwMap.get(DcMotor.class, "FrontLeftDrive");
        rightFrontDrive = hwMap.get(DcMotor.class, "FrontRightDrive");
        leftBackDrive   = hwMap.get(DcMotor.class, "BackLeftDrive");
        rightBackDrive  = hwMap.get(DcMotor.class, "BackRightDrive");
        
        // High speed motors for mechanisms
        intakeMotor    = hwMap.get(DcMotor.class, "IntakeMotor");
        
        // PID controlled motors
        launcherMotor  = hwMap.get(DcMotor.class, "LauncherMotor");   // 5203 series 6000rpm
        kickerMotor    = hwMap.get(DcMotor.class, "KickerMotor");     // 5203 series 312rpm
        
        // Initialize servos
        try {
            HoodServo      = hwMap.get(Servo.class, "HoodServo");
        } catch (Exception e) {
            HoodServo = null;
        }
        
        // Initialize IMU (optional - for field-centric drive)
        try {
            imu = hwMap.get(IMU.class, "IMU");
            
            // Initialize IMU with hub orientation
            // Adjust these values based on how your Control Hub is mounted:
            // LogoFacingDirection: Which way the REV logo faces
            // UsbFacingDirection: Which way the USB ports face
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,      // REV logo facing up
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD   // USB ports facing forward
            ));
            
            imu.initialize(parameters);
            imu.resetYaw(); // Reset heading to 0
            
        } catch (Exception e) {
            imu = null; // IMU not configured in robot config - field-centric will be disabled
        }
        
        // Set motor directions (adjust these based on your robot's wiring)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        
        // High speed motors - may need direction adjustment based on mounting
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        
        // Set all motors to zero power
        setDrivePower(0, 0, 0, 0);
        intakeMotor.setPower(0);
        
        // Set motor run modes
        // Precision drive motors (312 RPM) - excellent for controlled driving
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // High speed motors - good for fast mechanisms
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // PID motors setup - all need encoders for RPM control
        launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    
    /**
     * Set drive motor powers for mecanum drive
     * @param leftFront Left front motor power (-1.0 to 1.0)
     * @param rightFront Right front motor power (-1.0 to 1.0) 
     * @param leftBack Left back motor power (-1.0 to 1.0)
     * @param rightBack Right back motor power (-1.0 to 1.0)
     */
    public void setDrivePower(double leftFront, double rightFront, double leftBack, double rightBack) {
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }
    
    /**
     * Set tank drive powers (for simplified driving)
     * @param leftPower Left side power (-1.0 to 1.0)
     * @param rightPower Right side power (-1.0 to 1.0)
     */
    public void setTankDrive(double leftPower, double rightPower) {
        leftFrontDrive.setPower(leftPower);
        leftBackDrive.setPower(leftPower);
        rightFrontDrive.setPower(rightPower);
        rightBackDrive.setPower(rightPower);
    }
    
    /**
     * Mecanum drive calculation
     * @param drive Forward/backward movement
     * @param strafe Left/right movement  
     * @param twist Rotation
     * @param powerScale Overall power scaling (0.0 to 1.0)
     */
    public void mecanumDrive(double drive, double strafe, double twist, double powerScale) {
        double leftFrontPower = (drive + strafe + twist) * powerScale;
        double rightFrontPower = (drive - strafe - twist) * powerScale;
        double leftBackPower = (drive - strafe + twist) * powerScale;
        double rightBackPower = (drive + strafe - twist) * powerScale;
        
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }
    
    /**
     * Control intake with high speed motor
     * @param power Intake power (-1.0 to 1.0)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Get drive motor encoder positions (useful for autonomous)
     */
    public int[] getDriveEncoders() {
        return new int[]{
            leftFrontDrive.getCurrentPosition(),
            rightFrontDrive.getCurrentPosition(),
            leftBackDrive.getCurrentPosition(),
            rightBackDrive.getCurrentPosition()
        };
    }
    
    /**
     * Reset all drive encoders
     */
    public void resetDriveEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    
    /**
     * Check if IMU is available and working
     */
    public boolean isIMUAvailable() {
        return imu != null;
    }
    
    /**
     * Reset IMU heading to zero
     */
    public void resetIMUHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }
    
    /**
     * Get current robot heading in degrees
     */
    public double getHeadingDegrees() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        return 0.0;
    }
    
    /**
     * Get current robot heading in radians
     */
    public double getHeadingRadians() {
        if (imu != null) {
            return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        return 0.0;
    }
}
