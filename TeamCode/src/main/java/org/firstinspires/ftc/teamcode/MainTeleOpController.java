package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Main TeleOp Controller", group = "Competition")
public class MainTeleOpController extends LinearOpMode {

  // PID Motor State Variables
  private DcMotor launcherMotor = null;
  private double launcherTargetRPM = 4500.0;
  private double launcherCurrentRPM = 0.0;
  private double launcherIntegral = 0;
  private double launcherLastError = 0;
  private final ElapsedTime launcherTimer = new ElapsedTime();
  private boolean launcherActive = false;
  private int launcherLastPosition = 0;
  private double launcherLastTime = 0;

  private boolean yLauncherActive = false;
  private final ElapsedTime yButtonReleaseTimer = new ElapsedTime();
  private boolean yButtonReleased = false;

  private DcMotor intakeMotor = null;
  private static final double INTAKE_TARGET_RPM = 1000.0;
  private double intakeIntegral = 0;
  private double intakeLastError = 0;
  private final ElapsedTime intakeTimer = new ElapsedTime();
  private boolean intakeActive = false;
  private int intakeLastPosition = 0;
  private double intakeLastTime = 0;

  private DcMotor kickerMotor = null;
  private static final double KICKER_TARGET_RPM = 100.0;
  private double kickerIntegral = 0;
  private double kickerLastError = 0;
  private final ElapsedTime kickerTimer = new ElapsedTime();
  private boolean kickerActive = false;
  private int kickerLastPosition = 0;
  private double kickerLastTime = 0;

  // Servo State
  private Servo HoodServo = null;
  private double servoPosition = 0.5;

  @Override
  public void runOpMode() {
    // Drive constants
    final double NORMAL_SPEED = 0.8;
    final double PRECISION_SPEED = 0.4;
    final double ACCELERATION_LIMIT = 0.15;

    // PID constants
    final double LAUNCHER_KP = 0.01;
    final double LAUNCHER_KI = 0.001;
    final double LAUNCHER_KD = 0.0001;
    final double INTAKE_KP = 0.02;
    final double INTAKE_KI = 0.005;
    final double INTAKE_KD = 0.0001;
    final double KICKER_KP = 0.05;
    final double KICKER_KI = 0.01;
    final double KICKER_KD = 0.0005;

    // Servo constants
    final double SERVO_MIN_POSITION = 0.0;
    final double SERVO_MAX_POSITION = 1.0;

    // Hardware variables
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    IMU imu;
    NormalizedColorSensor colorSensor;
    final ElapsedTime runtime = new ElapsedTime();

    // Drive control state
    double currentMaxSpeed = NORMAL_SPEED;
    double lastLeftPower = 0;
    double lastRightPower = 0;
    double lastStrafePower = 0;

    // Control mode state
    boolean fieldRelative = false;
    boolean lastModeButton = false;
    boolean lastResetButton = false;

    // Intake motor toggle state
    boolean intakeToggleActive = false;
    boolean lastIntakeToggleButton = false;

    // Intake system kicker state
    boolean intakeKickerEnabled = true;
    boolean intakeKickerActive = false;

    // ===================== INITIALIZATION =====================
    leftFrontDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
    rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
    leftBackDrive = hardwareMap.get(DcMotor.class, "BackLeftDrive");
    rightBackDrive = hardwareMap.get(DcMotor.class, "BackRightDrive");

    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    try {
      imu = hardwareMap.get(IMU.class, "imu");
      telemetry.addData("IMU Status", "Connected - Field-centric available");
    } catch (Exception e) {
      imu = null;
      fieldRelative = false;
      telemetry.addData("IMU Status", "Not found - Robot-centric only");
    }

    try {
      colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
      if (colorSensor instanceof SwitchableLight) {
        ((SwitchableLight) colorSensor).enableLight(true);
      }
      telemetry.addData("Color Sensor", "Connected - REV Color Sensor V3");
    } catch (Exception e) {
      colorSensor = null;
      telemetry.addData("Color Sensor", "Not found - Color detection disabled");
    }

    try {
      HoodServo = hardwareMap.get(Servo.class, "HoodServo");
      HoodServo.setPosition(servoPosition);
      telemetry.addData("Hood Servo", "Connected - goBILDA servo ready");
    } catch (Exception e) {
      HoodServo = null;
      telemetry.addData("Hood Servo", "Not found - Servo control disabled");
    }

    try {
      launcherMotor = hardwareMap.get(DcMotor.class, "LauncherMotor");
      launcherMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      launcherTimer.reset();
      telemetry.addData("Launcher Motor", "Connected");
    } catch (Exception e) {
      launcherMotor = null;
      telemetry.addData("Launcher Motor", "Not found");
    }

    try {
      intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
      intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      intakeTimer.reset();
      telemetry.addData("Intake Motor", "Connected");
    } catch (Exception e) {
      intakeMotor = null;
      telemetry.addData("Intake Motor", "Not found");
    }

    try {
      kickerMotor = hardwareMap.get(DcMotor.class, "KickerMotor");
      kickerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      kickerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      kickerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      kickerTimer.reset();
      telemetry.addData("Kicker Motor", "Connected");
    } catch (Exception e) {
      kickerMotor = null;
      telemetry.addData("Kicker Motor", "Not found");
    }

    telemetry.addData("Status", "Main TeleOp Controller Ready");
    telemetry.update();

    waitForStart();
    runtime.reset();

    // ===================== MAIN CONTROL LOOP =====================
    while (opModeIsActive()) {
      // ================== SPEED MODE SELECTION ==================
      if (gamepad1.left_bumper) {
        currentMaxSpeed = PRECISION_SPEED;
      } else {
        currentMaxSpeed = NORMAL_SPEED;
      }

      // ================== FIELD RELATIVE TOGGLE ==================
      boolean currentModeButton = gamepad1.start;
      if (currentModeButton && !lastModeButton && imu != null) {
        fieldRelative = !fieldRelative;
      }
      lastModeButton = currentModeButton;

      // ================== IMU HEADING RESET ==================
      boolean currentResetButton = gamepad1.back;
      if (currentResetButton && !lastResetButton && imu != null) {
        imu.resetYaw();
      }
      lastResetButton = currentResetButton;

      // ================== DRIVETRAIN CONTROL ==================
      double rawDrive = -gamepad1.left_stick_y;
      double rawStrafe = gamepad1.left_stick_x;
      double rawTwist = gamepad1.right_stick_x;

      double drive = Math.abs(rawDrive) > 0.05 ? rawDrive : 0;
      double strafe = Math.abs(rawStrafe) > 0.05 ? rawStrafe : 0;
      double twist = Math.abs(rawTwist) > 0.05 ? rawTwist : 0;

      drive = cubicScale(drive);
      strafe = cubicScale(strafe);
      twist = cubicScale(twist) * 0.9;

      if (fieldRelative && imu != null) {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double robotHeading = orientation.getYaw(AngleUnit.RADIANS);
        double rotX = drive * Math.cos(-robotHeading) - strafe * Math.sin(-robotHeading);
        double rotY = drive * Math.sin(-robotHeading) + strafe * Math.cos(-robotHeading);
        drive = rotX;
        strafe = rotY;
      }

      double leftFrontPower = drive + strafe + twist;
      double rightFrontPower = drive - strafe - twist;
      double leftBackPower = drive - strafe + twist;
      double rightBackPower = drive + strafe - twist;

      double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
      maxPower = Math.max(maxPower, Math.abs(leftBackPower));
      maxPower = Math.max(maxPower, Math.abs(rightBackPower));

      if (maxPower > 1.0) {
        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;
      }

      leftFrontPower *= currentMaxSpeed;
      rightFrontPower *= currentMaxSpeed;
      leftBackPower *= currentMaxSpeed;
      rightBackPower *= currentMaxSpeed;

      leftFrontPower = applyAccelLimit(leftFrontPower, lastLeftPower, ACCELERATION_LIMIT);
      rightFrontPower = applyAccelLimit(rightFrontPower, lastRightPower, ACCELERATION_LIMIT);
      leftBackPower = applyAccelLimit(leftBackPower, lastStrafePower, ACCELERATION_LIMIT);
      rightBackPower =
          applyAccelLimit(
              rightBackPower, (lastLeftPower + lastRightPower + lastStrafePower) / 3.0, ACCELERATION_LIMIT);

      leftFrontDrive.setPower(leftFrontPower);
      rightFrontDrive.setPower(rightFrontPower);
      leftBackDrive.setPower(leftBackPower);
      rightBackDrive.setPower(rightBackPower);

      lastLeftPower = (leftFrontPower + leftBackPower) / 2.0;
      lastRightPower = (rightFrontPower + rightBackPower) / 2.0;
      lastStrafePower = (leftFrontPower + rightBackPower - leftBackPower - rightFrontPower) / 4.0;

      // ================== LAUNCHER CONTROL SECTION ==================
      boolean aButtonPressed = gamepad1.a;
      boolean yButtonPressed = gamepad1.y;

      if (aButtonPressed || yButtonPressed) {
        intakeKickerEnabled = true;

        if (aButtonPressed) {
          setLauncherActive(true);
          yLauncherActive = false;
          setServoPosition(0.0, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
          launcherTargetRPM = 4500.0;
          yButtonReleased = false;
        } else {
          setLauncherActive(true);
          yLauncherActive = true;
          setServoPosition(1.0, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
          launcherTargetRPM = 4750.0;
          yButtonReleased = false;
        }

        double launcherTolerance = launcherTargetRPM * 0.02;
        boolean launcherAtSpeed =
            Math.abs(launcherCurrentRPM - launcherTargetRPM) <= launcherTolerance;

        if (launcherAtSpeed) {
          setKickerActive(true);
          setIntakeActive(true);
        } else {
          setKickerActive(false);
          setIntakeActive(false);
        }

      } else {
        setLauncherActive(false);
        setKickerActive(false);
        setIntakeActive(false);
        intakeKickerActive = false;

        if (yLauncherActive && !yButtonReleased) {
          yButtonReleased = true;
          yButtonReleaseTimer.reset();
        }
        yLauncherActive = false;
      }

      if (yButtonReleased && yButtonReleaseTimer.seconds() >= 1.0) {
        if (!gamepad1.y && !gamepad1.a) {
          setServoPosition(0.0, SERVO_MIN_POSITION, SERVO_MAX_POSITION);
          yButtonReleased = false;
        }
      }

      if (gamepad1.right_bumper && !lastIntakeToggleButton) {
        intakeToggleActive = !intakeToggleActive;
        if (!gamepad1.a && !gamepad1.y) {
          if (intakeToggleActive) {
            setIntakeActive(true);
            if (intakeKickerEnabled) {
              intakeKickerActive = true;
              setKickerActive(true);
            }
          } else {
            setIntakeActive(false);
            intakeKickerActive = false;
            setKickerActive(false);
          }
        }
      }
      lastIntakeToggleButton = gamepad1.right_bumper;

      if (intakeToggleActive && !gamepad1.a && !gamepad1.y && intakeKickerActive) {
        if (colorSensor != null) {
          NormalizedRGBA colors = colorSensor.getNormalizedColors();
          boolean greenDetected =
              colors.green > colors.red && colors.green > colors.blue && colors.green > 0.3;
          boolean purpleDetected = colors.red > 0.3 && colors.blue > 0.3 && colors.green < 0.2;

          if (greenDetected || purpleDetected) {
            intakeKickerActive = false;
            intakeKickerEnabled = false;
            setKickerActive(false);
          }
        }
      }

      updateLauncherPID(LAUNCHER_KP, LAUNCHER_KI, LAUNCHER_KD);
      updateIntakePID(INTAKE_KP, INTAKE_KI, INTAKE_KD);
      updateKickerPID(KICKER_KP, KICKER_KI, KICKER_KD);

      // ================== TELEMETRY DISPLAY ==================
      telemetry.addData("Status", "Run Time: " + runtime.toString());
      telemetry.addData("Speed Mode", (gamepad1.left_bumper ? "PRECISION" : "NORMAL") + " (%.1f)", currentMaxSpeed);
      telemetry.addData("Drive Mode", fieldRelative ? "FIELD CENTRIC" : "ROBOT CENTRIC");

      if (imu != null) {
        telemetry.addData(
            "Robot Heading",
            "%.1f°",
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
      }

      // Color Sensor Telemetry
      if (colorSensor != null) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        String detectedColor = "Unknown";
        if (colors.red > 0.3 && colors.blue > 0.3 && colors.green < 0.2) {
          detectedColor = "Purple";
        } else if (colors.green > colors.red && colors.green > colors.blue && colors.green > 0.3) {
          detectedColor = "Green";
        }
        telemetry.addData("Detected Color", detectedColor);
      }

      // PID Motors Telemetry
      if (launcherMotor != null) {
        telemetry.addData(
            "Launcher",
            launcherActive
                ? String.format(Locale.US, "Active: %.0f/%.0f RPM", launcherCurrentRPM, launcherTargetRPM)
                : "Ready");
      }
      if (HoodServo != null) {
        telemetry.addData("Servo", "Position: %.2f", servoPosition);
      }

      telemetry.update();
    }
  }

  private double cubicScale(double input) {
    return input * input * input * 0.7 + input * 0.3;
  }

  private double applyAccelLimit(double targetPower, double currentPower, double limit) {
    double powerChange = targetPower - currentPower;
    if (Math.abs(powerChange) > limit) {
      return currentPower + (powerChange > 0 ? limit : -limit);
    }
    return targetPower;
  }

  private void setLauncherActive(boolean active) {
    if (launcherMotor == null) return;
    if (active && !launcherActive) {
      launcherActive = true;
      launcherIntegral = 0;
      launcherTimer.reset();
      launcherLastPosition = launcherMotor.getCurrentPosition();
      launcherLastTime = launcherTimer.seconds();
    } else if (!active && launcherActive) {
      launcherActive = false;
      launcherMotor.setPower(0);
    }
  }

  private void updateLauncherPID(double kP, double kI, double kD) {
    if (launcherMotor == null || !launcherActive) {
      launcherCurrentRPM = 0.0;
      return;
    }
    double currentTime = launcherTimer.seconds();
    double deltaTime = currentTime - launcherLastTime;
    if (deltaTime > 0) {
      int currentPosition = launcherMotor.getCurrentPosition();
      double deltaPosition = currentPosition - launcherLastPosition;
      launcherCurrentRPM = (deltaPosition / 1440.0 / deltaTime) * 60.0;
      launcherLastPosition = currentPosition;
    }
    launcherLastTime = currentTime;

    double error = launcherTargetRPM - launcherCurrentRPM;
    launcherIntegral += error * deltaTime;
    double derivative = (deltaTime > 0) ? (error - launcherLastError) / deltaTime : 0;
    launcherLastError = error;

    double pidOutput = kP * error + kI * launcherIntegral + kD * derivative;
    launcherMotor.setPower(Range.clip(pidOutput / 1000.0, -1.0, 1.0));
  }

  private void setIntakeActive(boolean active) {
    if (intakeMotor == null) return;
    if (active && !intakeActive) {
      intakeActive = true;
      intakeIntegral = 0;
      intakeTimer.reset();
      intakeLastPosition = intakeMotor.getCurrentPosition();
      intakeLastTime = intakeTimer.seconds();
    } else if (!active && intakeActive) {
      intakeActive = false;
      intakeMotor.setPower(0);
    }
  }

  private void updateIntakePID(double kP, double kI, double kD) {
    if (intakeMotor == null || !intakeActive) return;
    double currentTime = intakeTimer.seconds();
    double deltaTime = currentTime - intakeLastTime;
    double currentRPM = 0;
    if (deltaTime > 0) {
      int currentPosition = intakeMotor.getCurrentPosition();
      double deltaPosition = currentPosition - intakeLastPosition;
      currentRPM = (deltaPosition / 1440.0 / deltaTime) * 60.0;
      intakeLastPosition = currentPosition;
    }
    intakeLastTime = currentTime;

    double error = INTAKE_TARGET_RPM - currentRPM;
    intakeIntegral += error * deltaTime;
    double derivative = (deltaTime > 0) ? (error - intakeLastError) / deltaTime : 0;
    intakeLastError = error;

    double pidOutput = kP * error + kI * intakeIntegral + kD * derivative;
    intakeMotor.setPower(Range.clip(pidOutput / 1000.0, -1.0, 1.0));
  }

  private void setKickerActive(boolean active) {
    if (kickerMotor == null) return;
    if (active && !kickerActive) {
      kickerActive = true;
      kickerIntegral = 0;
      kickerTimer.reset();
      kickerLastPosition = kickerMotor.getCurrentPosition();
      kickerLastTime = kickerTimer.seconds();
    } else if (!active && kickerActive) {
      kickerActive = false;
      kickerMotor.setPower(0);
    }
  }

  private void updateKickerPID(double kP, double kI, double kD) {
    if (kickerMotor == null || !kickerActive) return;
    double currentTime = kickerTimer.seconds();
    double deltaTime = currentTime - kickerLastTime;
    double currentRPM = 0;
    if (deltaTime > 0) {
      int currentPosition = kickerMotor.getCurrentPosition();
      double deltaPosition = currentPosition - kickerLastPosition;
      currentRPM = (deltaPosition / 1440.0 / deltaTime) * 60.0;
      kickerLastPosition = currentPosition;
    }
    kickerLastTime = currentTime;

    double error = KICKER_TARGET_RPM - currentRPM;
    kickerIntegral += error * deltaTime;
    double derivative = (deltaTime > 0) ? (error - kickerLastError) / deltaTime : 0;
    kickerLastError = error;

    double pidOutput = kP * error + kI * kickerIntegral + kD * derivative;
    kickerMotor.setPower(Range.clip(pidOutput / 1000.0, -1.0, 1.0));
  }

  private void setServoPosition(double position, double min, double max) {
    if (HoodServo == null) return;
    servoPosition = Range.clip(position, min, max);
    HoodServo.setPosition(servoPosition);
  }
}
