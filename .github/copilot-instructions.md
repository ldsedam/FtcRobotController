# FTC Robot Controller - AI Coding Agent Guide

## Project Overview

This is a FIRST Tech Challenge (FTC) robotics codebase for the DECODE (2025-2026) season. The project uses the official FTC SDK v11.0+ with Android Studio and Java for robot control software.

**Competition Context**: FTC robots operate in two modes:
- **Autonomous** (30 seconds): Pre-programmed movement and actions
- **TeleOp** (2 minutes): Human-controlled via gamepad

## Critical Architecture Patterns

### Hardware Abstraction Layer
All robot hardware is centralized in `RobotHardware.java` (in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`):
```java
// Hardware is initialized once and shared across OpModes
RobotHardware robot = new RobotHardware();
robot.init(hardwareMap);  // Called in OpMode initialization
```

**Key Hardware Components**:
- **Drive motors**: 312 RPM goBILDA precision motors (mecanum drive: `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`)
- **PID-controlled motors**: 3 motors with precise RPM control (`launcher_motor` @ 4500 RPM, `pickup_motor` @ 100 RPM, `kicker_motor` @ 150 RPM)
- **IMU**: Built-in Control Hub sensor for field-centric navigation (reset heading with `imu.resetYaw()`)
- **REV Color Sensor V3**: Object detection with LED control (`color_sensor`)
- **Servos**: goBILDA Torque Servo for precise positioning

### OpMode Organization
OpModes are the executable robot programs. They use annotations for Robot Controller app recognition:

```java
@TeleOp(name="Display Name", group="Category")  // Driver-controlled mode
@Autonomous(name="Display Name", group="Category")  // Pre-programmed mode
public class MyOpMode extends LinearOpMode { /* ... */ }
```

**Competition-Ready OpModes** (in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`):
- `MainTeleOpController.java`: Primary driver control with field-centric drive, PID motors, color sensor
- `MainAutonomous.java`: Template autonomous with encoder-based movement and PID motor control
- `DecodeRedSideAutonomous.java`: Competition-specific autonomous for red alliance preload strategy

**Testing OpModes**:
- `AdvancedMecanumDrive.java`: Field-centric mecanum drive testing with acceleration limiting
- `PIDMotorTest.java`: Individual PID motor tuning and debugging
- `IMUTest.java`, `MotorTest.java`: Hardware validation

### PID Motor Control Pattern
Three motors use custom PID controllers for precise RPM control (NOT built-in FTC PID):

```java
// PID calculation runs in loop() - update every iteration
double currentRPM = calculateRPM(motor.getCurrentPosition(), lastPosition, elapsedTime);
double error = targetRPM - currentRPM;
integral += error * dt;
double derivative = (error - lastError) / dt;
double power = (kP * error) + (kI * integral) + (kD * derivative);
motor.setPower(Range.clip(power, 0, 1.0));  // Clamp to [0, 1]
```

**Tuning Constants** (in respective OpModes):
- Launcher: kP=0.01, kI=0.001, kD=0.0001 (high-speed, 4500 RPM)
- Pickup: kP=0.02, kI=0.005, kD=0.0001 (low-speed, 100 RPM)
- Kicker: kP=0.05, kI=0.01, kD=0.0005 (precision motor, 150 RPM)

### Field-Centric Drive Implementation
The robot can drive relative to the field (not robot orientation) using IMU heading:

```java
double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
// Then calculate mecanum drive powers with rotX, rotY
```

Toggle with START button in TeleOp. Reset heading with BACK button (always use BACK for heading reset, not manual recalibration).

### Coordinate System for Autonomous
Position-based autonomous uses field coordinates with origin at field center (72", 72" from corner):
- **X-axis**: Right = positive (0 to 144")
- **Y-axis**: Forward (away from driver) = positive (0 to 144")
- **Heading**: 0° = forward, 90° = right, 180° = backward, 270° = left

See `FIELD_COORDINATE_GUIDE.md` for game element positions and `PositionSteeringAutonomous.java` for implementation.

## Development Workflow

### Building and Deploying
**Using Android Studio** (required for code changes):
1. Open project in Android Studio Ladybug (2024.2) or later
2. Connect to Robot Controller via WiFi Direct or USB (enable ADB over WiFi)
3. Build: `Build > Make Project` or Ctrl+F9
4. Deploy: `Run > Run 'TeamCode'` or Shift+F10
5. Select Robot Controller device when prompted

**Gradle Build Commands** (Windows PowerShell):
```powershell
.\gradlew assembleDebug          # Build APK
.\gradlew :TeamCode:build        # Build TeamCode module only
```

**DO NOT** use `./gradlew` (Linux syntax) - this is Windows environment.

### Testing Protocol
1. **Hardware validation**: Run `MotorTest.java` or `IMUTest.java` first for any new hardware
2. **PID tuning**: Use `PIDMotorTest.java` with single button controls (A, Y, X) to tune each motor independently
3. **Integration test**: Run `MainTeleOpController.java` for full system test
4. **Autonomous validation**: Test autonomous in controlled space before competition

### Key Configuration Files
- `TeamCode/src/main/res/xml/robot_config.xml` (generated by Driver Station): Hardware device names and ports
- Motor/sensor names in code MUST match configuration file exactly (case-sensitive)
- Common config names: `left_front_drive`, `launcher_motor`, `color_sensor`, `imu`

## Critical Conventions

### Motor Direction and Zero Power Behavior
```java
// Set direction based on physical mounting (test with MotorTest.java)
motor.setDirection(DcMotor.Direction.FORWARD);  // or REVERSE

// For PID motors, use FLOAT to allow free movement when unpowered
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // PID motors
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // Drive motors
```

### Encoder Usage
```java
// Reset before autonomous movements
motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // PID-assisted power control

// For precise autonomous positioning
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setTargetPosition(encoderTarget);
```

**Encoder Counts**: goBILDA motors use ~1120 ticks per revolution. Calculate `COUNTS_PER_INCH` based on wheel diameter.

### Gamepad Button Handling
```java
// Avoid repeated triggers with state tracking
if (gamepad1.a && !lastAButton) {
    // Button just pressed (rising edge)
}
lastAButton = gamepad1.a;
```

Use `gamepad1` for driver, `gamepad2` for operator. See `MAIN_TELEOP_CONTROLS.md` for button mappings.

### Telemetry Best Practices
```java
telemetry.addData("Label", "Value: %.2f", variable);  // Format numbers
telemetry.addLine("--- Section Header ---");          // Group related data
telemetry.update();  // Call once per loop iteration
```

## Common Pitfalls

1. **Hardware name mismatches**: Code references must EXACTLY match Driver Station configuration (e.g., `launcher_motor` ≠ `launcherMotor`)
2. **PID motor runaway**: Always clamp PID output with `Range.clip(power, 0, 1.0)` and reset integral on mode changes
3. **IMU drift**: Reset heading with BACK button at start of each match
4. **Encoder overflow**: Use `getCurrentPosition()` frequently; don't let calculations span too many loop iterations
5. **Android-specific imports**: Use `com.qualcomm.robotcore.*` for FTC SDK, NOT standard Java libraries for hardware

## Documentation Map

Extensive markdown guides in `TeamCode/` provide step-by-step instructions:
- `ROBOT_SETUP_GUIDE.md`: Hardware configuration and motor specifications
- `MAIN_TELEOP_CONTROLS.md`: Complete gamepad control reference
- `FIELD_COORDINATE_GUIDE.md`: Autonomous positioning system
- `*_COMPLETE.md` files: Feature-specific implementation guides (servos, PID, color sensor, etc.)

## External Dependencies

- **FTC SDK v11.0+**: Official libraries in `ftc-sdk-stubs/` (read-only reference)
- **Gradle**: Defined in `build.dependencies.gradle` - do not modify without team coordination
- **Android API Level**: Targets Android SDK specified in build files (typically API 29-34)

When adding new features, follow the established pattern: create/update hardware in `RobotHardware.java`, implement logic in OpModes, document controls in markdown guides.
