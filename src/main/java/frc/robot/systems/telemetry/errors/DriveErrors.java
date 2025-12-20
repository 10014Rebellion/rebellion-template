// REBELLION 10014

package frc.robot.systems.telemetry.errors;

import frc.robot.systems.telemetry.TelemetryConstants.Severity;
import frc.robot.systems.telemetry.TelemetryError;

public class DriveErrors {
    public record ProfileExponentZero(int pDefaultExponent) implements TelemetryError {
        @Override
        public String message() {
            return "DRIVE EXPONENT SET TO ZERO, FIX THE PROFILE, DEFAULTING TO input^" + pDefaultExponent;
        }

        @Override
        public Severity severity() {
            return Severity.ERROR;
        }
    }
}
