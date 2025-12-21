// REBELLION 10014

package frc.robot.bindings;

import frc.robot.systems.drive.controllers.ManualTeleopController.DriverProfiles;

public class BindingsConstants {
    public static final int kDriverControllerPort = 0;

    public static final DriverProfiles kDefaultProfile =
            new DriverProfiles(1, 3, 0.075, 1.0, 3.0, 0.1, 0.2, true, "Default");

    public static DriverProfiles[] kProfiles = {
        kDefaultProfile, new DriverProfiles(1, 3, 0.075, 0.8, 3.0, 0.1, 0.2, false, "Taha The GOAT")
    };
}
