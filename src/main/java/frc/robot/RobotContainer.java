// REBELLION 10014

package frc.robot;

import static frc.robot.systems.drive.DriveConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.GyroIO;
import frc.robot.systems.drive.GyroIOPigeon2;
import frc.robot.systems.drive.Module;
import frc.robot.systems.drive.ModuleIO;
import frc.robot.systems.drive.ModuleIOFXFXS;
import frc.robot.systems.drive.ModuleIOSim;
import frc.robot.systems.vision.CameraIO;
import frc.robot.systems.vision.CameraIOPV;
import frc.robot.systems.vision.Vision;
import frc.robot.systems.vision.VisionConstants;
import frc.robot.systems.vision.VisionConstants.Orientation;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private final Drive mDrive;

    private final LoggedDashboardChooser<Command> driverProfileChooser = new LoggedDashboardChooser<>("DriverProfile");

    public RobotContainer() {
        new StateTracker();

        switch (Constants.currentMode) {
            case REAL:
                mDrive = new Drive(
                        new Module[] {
                            new Module("FL", new ModuleIOFXFXS(kFrontLeftHardware)),
                            new Module("FR", new ModuleIOFXFXS(kFrontRightHardware)),
                            new Module("BL", new ModuleIOFXFXS(kBackLeftHardware)),
                            new Module("BR", new ModuleIOFXFXS(kBackRightHardware))
                        },
                        new GyroIOPigeon2(),
                        new Vision(new CameraIO[] {
                            new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.BACK),
                            new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.BACK)
                        }));
                break;

            case SIM:
                mDrive = new Drive(
                        new Module[] {
                            new Module("FL", new ModuleIOSim()),
                            new Module("FR", new ModuleIOSim()),
                            new Module("BL", new ModuleIOSim()),
                            new Module("BR", new ModuleIOSim())
                        },
                        new GyroIO() {},
                        new Vision(new CameraIO[] {
                            new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, Orientation.BACK),
                            new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, Orientation.BACK)
                        }));
                break;

            default:
                mDrive = new Drive(
                        new Module[] {
                            new Module("FL", new ModuleIO() {}),
                            new Module("FR", new ModuleIO() {}),
                            new Module("BL", new ModuleIO() {}),
                            new Module("BR", new ModuleIO() {})
                        },
                        new GyroIO() {},
                        new Vision(new CameraIO[] {new CameraIO() {}, new CameraIO() {}}));
                break;
        }

        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    public Drive getDrivetrain() {
        return mDrive;
    }

    public Command getAutonomousCommand() {
        return null;
    }

    public Command getDriverProfileCommand() {
        return driverProfileChooser.get();
    }
}
