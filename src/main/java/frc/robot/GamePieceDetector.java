package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;

import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Optional;

public class GamePieceDetector {
    private final SerialPort serialPort = new SerialPort(115200, SerialPort.Port.kMXP, 8, SerialPort.Parity.kNone);

    public Optional<Integer> detect() {
        var reading = serialPort.read(serialPort.getBytesReceived());
        var readingString = new String(reading, StandardCharsets.UTF_8);

        if (!readingString.endsWith("\n"))
            return Optional.empty();

        var readings =
            Arrays
                .stream(readingString.split("[\r\n]"))
                .filter(s -> !s.isEmpty())
                .toArray(String[]::new);

        if (readings.length == 0)
            return Optional.empty();

        try {
            var value = Integer.valueOf(readings[readings.length - 1]);
            return Optional.of(value);
        } catch (NumberFormatException e) {
            var mode = DriverStation.isAutonomous() ? "Autonomous"
                : DriverStation.isTeleop() ? "Teleop"
                : DriverStation.isTest() ? "Test"
                : "Unknown";
            System.out.printf(
                "HANDLED ERROR - %s %f: %s%n",
                mode,
                DriverStation.getMatchTime(),
                e.getMessage());
            return Optional.empty();
        }

    }
}
