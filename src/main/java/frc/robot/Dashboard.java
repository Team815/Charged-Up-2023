package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.input.InputDevice;
import frc.robot.input.Joystick;
import frc.robot.input.XboxController;
import frc.robot.subsystems.GyroAngles;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Map;
import java.util.function.Supplier;

public final class Dashboard {
    private Dashboard() {
        throw new AssertionError("utility class");
    }

    public static void createSwerveDriveLayout(String tabName, int column, int row, SwerveDrive swerveDrive) {
        var inst = NetworkTableInstance.getDefault();
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Swerve Drive", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 4));

        var autoCorrectEnabledEntry = layout
            .add("Auto Correct", SwerveDrive.DEFAULT_AUTO_CORRECT_ENABLED)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .getEntry();
        inst.addListener(
            autoCorrectEnabledEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> swerveDrive.setAutoCorrectEnabled(e.valueData.value.getBoolean()));

        var autoCorrectDelayEntry = layout
            .add("Auto Correct Delay", SwerveDrive.DEFAULT_AUTO_CORRECT_DELAY)
            .withPosition(0, 1)
            .getEntry();
        inst.addListener(
            autoCorrectDelayEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> swerveDrive.setAutoCorrectDelay(e.valueData.value.getDouble()));

        var maxLinearAccelerationEntry = layout
            .add("Max Linear Acceleration", SwerveDrive.DEFAULT_MAX_LINEAR_ACCELERATION)
            .withPosition(0, 2)
            .getEntry();
        inst.addListener(
            maxLinearAccelerationEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> swerveDrive.setMaxLinearAcceleration(e.valueData.value.getDouble()));

        var maxAngularAccelerationEntry = layout
            .add("Max Angular Acceleration", SwerveDrive.DEFAULT_MAX_ANGULAR_ACCELERATION)
            .withPosition(0, 3)
            .getEntry();
        inst.addListener(
            maxAngularAccelerationEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> swerveDrive.setMaxAngularAcceleration(e.valueData.value.getDouble()));
    }

    public static void createSwerveModuleLayout(String tabName, int column, int row, SwerveModule... swerveModules) {
        var inst = NetworkTableInstance.getDefault();
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Swerve Modules", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 5));

        var maxLinearSpeedEntry = layout
            .add("Max Linear Speed", SwerveModule.DEFAULT_MAX_LINEAR_SPEED)
            .withPosition(0, 0)
            .getEntry();
        inst.addListener(
            maxLinearSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> Arrays.stream(swerveModules).forEach(module -> module.setMaxLinearSpeed(e.valueData.value.getDouble())));

        var maxAngularSpeedEntry = layout
            .add("Max Angular Speed", SwerveModule.DEFAULT_MAX_ANGULAR_SPEED)
            .withPosition(0, 1)
            .getEntry();
        inst.addListener(
            maxAngularSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> Arrays.stream(swerveModules).forEach(module -> module.setMaxAngularSpeed(e.valueData.value.getDouble())));

        var maxLinearRateEntry = layout
            .add("Max Linear Acceleration", SwerveModule.DEFAULT_MAX_LINEAR_ACCELERATION)
            .withPosition(0, 2)
            .getEntry();
        inst.addListener(
            maxLinearRateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> Arrays.stream(swerveModules).forEach(module -> module.setMaxLinearAcceleration(e.valueData.value.getDouble())));

        var maxAngularRateEntry = layout
            .add("Max Angular Acceleration", SwerveModule.DEFAULT_MAX_ANGULAR_ACCELERATION)
            .withPosition(0, 3)
            .getEntry();
        inst.addListener(
            maxAngularRateEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> Arrays.stream(swerveModules).forEach(module -> module.setMaxAngularAcceleration(e.valueData.value.getDouble())));

        var pEntry = layout
            .add("Angular P", SwerveModule.DEFAULT_P)
            .withPosition(0, 4)
            .getEntry();
        inst.addListener(
            pEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> Arrays.stream(swerveModules).forEach(module -> module.setP(e.valueData.value.getDouble())));
    }

    public static void createLimelightLayout(String tabName, int column, int row, GamePieceLimelight limelight) {
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Limelight", BuiltInLayouts.kGrid)
            .withSize(2, 1)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

        layout
            .addString("Target", limelight::getTarget)
            .withPosition(0, 0);
    }

    public static void createControllerLayout(String tabName, int column, int row, Supplier<InputDevice> inputDeviceSupplier, RobotContainer container) {
        var inst = NetworkTableInstance.getDefault();
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Teleop", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 4));

        var inputDeviceChoiceEntry = layout
            .add("Joystick <-> Xbox Controller", true)
            .withWidget(BuiltInWidgets.kToggleSwitch)
            .withPosition(0, 0)
            .getEntry();
        inst.addListener(
            inputDeviceChoiceEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> {
                var useXboxController = e.valueData.value.getBoolean();
                container.setInputDevice(useXboxController ? new XboxController() : new Joystick());
            });

        var maxTeleopXSpeedEntry = layout
            .add("Max Forward Speed", InputDevice.DEFAULT_MAX_FORWARD_SPEED)
            .withPosition(0, 1)
            .getEntry();
        inst.addListener(
            maxTeleopXSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> inputDeviceSupplier.get().setMaxForwardSpeed(e.valueData.value.getDouble()));

        var maxTeleopYSpeedEntry = layout
            .add("Max Sideways Speed", InputDevice.DEFAULT_MAX_SIDEWAYS_SPEED)
            .withPosition(0, 2)
            .getEntry();
        inst.addListener(
            maxTeleopYSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> inputDeviceSupplier.get().setMaxSidewaysSpeed(e.valueData.value.getDouble()));

        var maxTeleopAngularSpeedEntry = layout
            .add("Max Angular Speed", InputDevice.DEFAULT_MAX_ANGULAR_SPEED)
            .withPosition(0, 3)
            .getEntry();
        inst.addListener(
            maxTeleopAngularSpeedEntry,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> inputDeviceSupplier.get().setMaxAngularSpeed(e.valueData.value.getDouble()));
    }

    public static void createPoseLayout(String tabName, int column, int row, Supplier<Pose2d> pose) {
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Pose", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

        layout
            .addString("Forward Position", () -> String.format("%.2f", pose.get().getX()))
            .withPosition(0, 0);
        layout
            .addString("Sideways Position", () -> String.format("%.2f", pose.get().getY()))
            .withPosition(0, 1);
        layout
            .addString("Angular Position", () -> String.format("%.2f", pose.get().getRotation().getDegrees()))
            .withPosition(0, 2);
    }

    public static void createAnglesLayout(String tabName, int column, int row, Supplier<GyroAngles> angles) {
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Angles", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

        layout
            .addString("Pitch", () -> String.format("%.2f", angles.get().getPitch()))
            .withPosition(0, 0);
        layout
            .addString("Roll", () -> String.format("%.2f", angles.get().getRoll()))
            .withPosition(0, 1);
        layout
            .addString("Yaw", () -> String.format("%.2f", angles.get().getYaw()))
            .withPosition(0, 2);
    }

    public static void createVelocityLayout(String tabName, int column, int row, Supplier<ChassisSpeeds> speeds) {
        var layout = Shuffleboard.getTab(tabName)
            .getLayout("Velocity", BuiltInLayouts.kGrid)
            .withSize(2, 2)
            .withPosition(column, row)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 3));

        layout
            .addString("Forward Velocity", () -> String.format("%.2f", speeds.get().vxMetersPerSecond))
            .withPosition(0, 0);
        layout
            .addString("Sideways Velocity", () -> String.format("%.2f", speeds.get().vyMetersPerSecond))
            .withPosition(0, 1);
        layout
            .addString("Angular Velocity", () -> String.format("%.2f", speeds.get().omegaRadiansPerSecond))
            .withPosition(0, 2);
    }
}
