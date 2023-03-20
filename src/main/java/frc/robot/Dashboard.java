package frc.robot;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.input.InputDevice;
import frc.robot.input.Joystick;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;

import java.util.Arrays;
import java.util.EnumSet;
import java.util.Map;
import java.util.function.Supplier;

public class Dashboard {
    public Dashboard(String tabName, SwerveDrive swerveDrive, GamePieceLimelight limelight, Supplier<InputDevice> inputDeviceSupplier, RobotContainer container, SwerveModule... swerveModules) {
        var tab = Shuffleboard.getTab(tabName);
        var inst = NetworkTableInstance.getDefault();
        createSwerveModuleLayout(tab, inst, swerveModules);
        createSwerveDriveLayout(tab, inst, swerveDrive);
        createLimelightLayout(tab, limelight);
        createControllerLayout(tab, inst, inputDeviceSupplier, container);
    }

    private void createSwerveDriveLayout(ShuffleboardTab tab, NetworkTableInstance inst, SwerveDrive swerveDrive) {
        var layout = tab
            .getLayout("Swerve Drive", BuiltInLayouts.kGrid)
            .withSize(2, 2)
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

    private void createSwerveModuleLayout(ShuffleboardTab tab, NetworkTableInstance inst, SwerveModule[] swerveModules) {
        var layout = tab
            .getLayout("Swerve Modules", BuiltInLayouts.kGrid)
            .withSize(2, 2)
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

    private void createLimelightLayout(ShuffleboardTab tab, GamePieceLimelight limelight) {
        var layout = tab
            .getLayout("Limelight", BuiltInLayouts.kGrid)
            .withSize(2, 1)
            .withProperties(Map.of("Label position", "LEFT", "Number of columns", 1, "Number of rows", 2));

        layout
            .addString("Target", limelight::getTarget)
            .withPosition(0, 0);
    }

    private void createControllerLayout(ShuffleboardTab tab, NetworkTableInstance inst, Supplier<InputDevice> inputDeviceSupplier, RobotContainer container) {
        var layout = tab
            .getLayout("Teleop", BuiltInLayouts.kGrid)
            .withSize(2, 2)
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
}