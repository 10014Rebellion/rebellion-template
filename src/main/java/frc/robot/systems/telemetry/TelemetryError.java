// REBELLION 10014

package frc.robot.systems.telemetry;

import frc.robot.systems.telemetry.TelemetryConstants.Severity;

public interface TelemetryError {
    String message();

    Severity severity();
}
