package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name = "Position Steering Autonomous", group = "Competition")
public class PositionSteeringAutonomous extends LinearOpMode {

  // Hardware
  private DcMotor leftFrontDrive = null;
  private DcMotor rightFrontDrive = null;
  private DcMotor leftBackDrive = null;
  private DcMotor rightBackDrive = null;
  private IMU imu = null;
  private Limelight3A limelight = null;
  private DcMotor launcherMotor = null;
  private DcMotor intakeMotor = null;
  private DcMotor kickerMotor = null;

  // Position State
  private double currentX = 0.0;
  private double currentY = 0.0;
  private double currentHeading = 0.0;
  private boolean limelightAvailable = false;
  private boolean imuAvailable = false;

  // Navigation Constants
  private static final double MAX_DRIVE_SPEED = 0.8;
  private static final double MIN_DRIVE_SPEED = 0.15;
  private static final double MAX_TURN_SPEED = 0.6;
  private static final double POSITION_TOLERANCE = 2.0;
  private static final double HEADING_TOLERANCE = 3.0;

  // PID Constants
  private static final double POSITION_KP = 0.03;
  private static final double POSITION_KI = 0.001;
  private static final double POSITION_KD = 0.01;
  private static final double HEADING_KP = 0.02;
  private static final double HEADING_KI = 0.0005;
  private static final double HEADING_KD = 0.005;

  // PID State
  private double positionIntegral = 0;
  private double positionLastError = 0;
  private double headingIntegral = 0;
  private double headingLastError = 0;

  // Dead Reckoning State
  private int lastLeftFront = 0;
  private int lastRightFront = 0;
  private int lastLeftBack = 0;
  private int lastRightBack = 0;
  private static final double COUNTS_PER_INCH = 1120 / (4 * Math.PI);

  // Action Definitions
  public static final int ACTION_LAUNCH_STANDARD = 1;
  public static final int ACTION_LAUNCH_HIGH_POWER = 2;
  public static final int ACTION_PICKUP_SEQUENCE = 3;
  public static final int ACTION_WAIT = 4;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initializing...");
    telemetry.update();

    initializeHardware();
    setStartingPosition(0, 0, 0);

    telemetry.addData("Status", "Ready");
    telemetry.update();

    waitForStart();

    if (opModeIsActive()) {
      // Example Autonomous Sequence
      moveToPosition(24, 36, 0);
      executeAction(ACTION_LAUNCH_STANDARD);
      moveToPosition(48, 24, 90);
      executeAction(ACTION_PICKUP_SEQUENCE);
      moveToPosition(12, 48, 180);
      executeAction(ACTION_LAUNCH_HIGH_POWER);
      moveToPosition(6, 6, 270);
    }
  }

  private void initializeHardware() {
    leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
    leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
    rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    resetDriveEncoders();
    setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

    try {
      imu = hardwareMap.get(IMU.class, "imu");
      imu.resetYaw();
      imuAvailable = true;
    } catch (Exception e) {
      imuAvailable = false;
    }

    try {
      limelight = hardwareMap.get(Limelight3A.class, "limelight");
      limelight.pipelineSwitch(0);
      limelight.start();
      limelightAvailable = true;
    } catch (Exception e) {
      limelightAvailable = false;
    }

    try {
      launcherMotor = hardwareMap.get(DcMotor.class, "LauncherMotor");
      intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
      kickerMotor = hardwareMap.get(DcMotor.class, "KickerMotor");
    } catch (Exception e) {
      // Action hardware missing
    }
  }

  public void setStartingPosition(double x, double y, double heading) {
    currentX = x;
    currentY = y;
    currentHeading = heading;
    if (imuAvailable) {
      imu.resetYaw();
    }
  }

  public void moveToPosition(double targetX, double targetY, double targetHeading) {
    positionIntegral = 0;
    positionLastError = 0;
    headingIntegral = 0;
    headingLastError = 0;

    ElapsedTime moveTimer = new ElapsedTime();
    while (opModeIsActive() && moveTimer.seconds() < 10.0) {
      updatePosition();

      double deltaX = targetX - currentX;
      double deltaY = targetY - currentY;
      double distanceError = Math.hypot(deltaX, deltaY);
      double headingError = normalizeAngle(targetHeading - currentHeading);

      if (distanceError < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
        break;
      }

      double desiredAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
      double robotRelativeAngle = normalizeAngle(desiredAngle - currentHeading);
      double robotRelativeAngleRad = Math.toRadians(robotRelativeAngle);

      double positionOutput = calculatePositionPID(distanceError);
      double drive = Math.cos(robotRelativeAngleRad) * positionOutput;
      double strafe = Math.sin(robotRelativeAngleRad) * positionOutput;
      double twist = calculateHeadingPID(headingError);

      double leftFrontPower = drive + strafe + twist;
      double rightFrontPower = drive - strafe - twist;
      double leftBackPower = drive - strafe + twist;
      double rightBackPower = drive + strafe - twist;

      double maxPower = Math.max(1.0, Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));
      maxPower = Math.max(maxPower, Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)));

      leftFrontDrive.setPower(leftFrontPower / maxPower * MAX_DRIVE_SPEED);
      rightFrontDrive.setPower(rightFrontPower / maxPower * MAX_DRIVE_SPEED);
      leftBackDrive.setPower(leftBackPower / maxPower * MAX_DRIVE_SPEED);
      rightBackDrive.setPower(rightBackPower / maxPower * MAX_DRIVE_SPEED);

      sleep(20);
    }
    stopDriveMotors();
  }

  public void executeAction(int actionType) {
    switch (actionType) {
      case ACTION_LAUNCH_STANDARD:
        performLauncherSequence();
        break;
      case ACTION_LAUNCH_HIGH_POWER:
        performLauncherSequence();
        break;
      case ACTION_PICKUP_SEQUENCE:
        performPickupSequence();
        break;
      case ACTION_WAIT:
        sleep(1000);
        break;
    }
  }

  private void updatePosition() {
    if (limelightAvailable && updatePositionFromLimelight()) {
      return;
    }
    updatePositionFromDeadReckoning();
  }

  private boolean updatePositionFromLimelight() {
    try {
      LLResult result = limelight.getLatestResult();
      if (result != null && result.isValid()) {
        Pose3D robotPose = result.getBotpose();
        if (robotPose != null) {
          currentX = 0; // Placeholder: robotPose.x
          currentY = 0; // Placeholder: robotPose.y
          currentHeading = 0; // Placeholder: robotPose.heading
          return true;
        }
      }
    } catch (Exception e) {
      // Fallback to dead reckoning
    }
    return false;
  }

  private void updatePositionFromDeadReckoning() {
    int leftFrontPos = leftFrontDrive.getCurrentPosition();
    int rightFrontPos = rightFrontDrive.getCurrentPosition();
    int leftBackPos = leftBackDrive.getCurrentPosition();
    int rightBackPos = rightBackDrive.getCurrentPosition();

    double deltaLeft =
        ((leftFrontPos - lastLeftFront) + (leftBackPos - lastLeftBack)) / 2.0 / COUNTS_PER_INCH;
    double deltaRight =
        ((rightFrontPos - lastRightFront) + (rightBackPos - lastRightBack)) / 2.0 / COUNTS_PER_INCH;
    double deltaForward = (deltaLeft + deltaRight) / 2.0;
    double deltaStrafe =
        ((leftFrontPos - lastLeftFront)
                - (leftBackPos - lastLeftBack)
                + (rightBackPos - lastRightBack)
                - (rightFrontPos - lastRightFront)) / 4.0
            / COUNTS_PER_INCH;

    if (imuAvailable) {
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      currentHeading = orientation.getYaw(AngleUnit.DEGREES);
    }

    double headingRad = Math.toRadians(currentHeading);
    currentX += deltaForward * Math.cos(headingRad) - deltaStrafe * Math.sin(headingRad);
    currentY += deltaForward * Math.sin(headingRad) + deltaStrafe * Math.cos(headingRad);

    lastLeftFront = leftFrontPos;
    lastRightFront = rightFrontPos;
    lastLeftBack = leftBackPos;
    lastRightBack = rightBackPos;
  }

  private double calculatePositionPID(double error) {
    positionIntegral += error;
    double derivative = error - positionLastError;
    positionLastError = error;
    positionIntegral = Range.clip(positionIntegral, -100, 100);
    double output = POSITION_KP * error + POSITION_KI * positionIntegral + POSITION_KD * derivative;
    return Range.clip(output, MIN_DRIVE_SPEED, MAX_DRIVE_SPEED);
  }

  private double calculateHeadingPID(double error) {
    headingIntegral += error;
    double derivative = error - headingLastError;
    headingLastError = error;
    headingIntegral = Range.clip(headingIntegral, -100, 100);
    double output = HEADING_KP * error + HEADING_KI * headingIntegral + HEADING_KD * derivative;
    return Range.clip(output, -MAX_TURN_SPEED, MAX_TURN_SPEED);
  }

  private void performLauncherSequence() {
    if (launcherMotor != null) {
      launcherMotor.setPower(1.0);
    }
    sleep(2000);
    if (intakeMotor != null) {
      intakeMotor.setPower(0.8);
    }
    if (kickerMotor != null) {
      kickerMotor.setPower(0.3);
    }
    sleep(3000);
    if (launcherMotor != null) launcherMotor.setPower(0);
    if (intakeMotor != null) intakeMotor.setPower(0);
    if (kickerMotor != null) kickerMotor.setPower(0);
  }

  private void performPickupSequence() {
    if (intakeMotor != null) {
      intakeMotor.setPower(0.6);
    }
    sleep(2000);
    if (intakeMotor != null) {
      intakeMotor.setPower(0);
    }
  }

  private double normalizeAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle <= -180) angle += 360;
    return angle;
  }

  private void resetDriveEncoders() {
    leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  private void setDriveMode(DcMotor.RunMode mode) {
    leftFrontDrive.setMode(mode);
    rightFrontDrive.setMode(mode);
    leftBackDrive.setMode(mode);
    rightBackDrive.setMode(mode);
  }

  private void stopDriveMotors() {
    leftFrontDrive.setPower(0);
    rightFrontDrive.setPower(0);
    leftBackDrive.setPower(0);
    rightBackDrive.setPower(0);
  }
}
