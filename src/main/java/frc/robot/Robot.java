// REBELLION 10014

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DashboardConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // start dashboard webserver
        if (DashboardConstants.DASHBOARD_ENABLED) {
            System.out.println("Starting Dashboard Webserver");
            System.out.println("Dashboard directory: "
                    + Filesystem.getDeployDirectory()
                    + "/"
                    + DashboardConstants.DASHBOARD_PATH);
            try {
                WebServer.start(
                        DashboardConstants.DASHBOARD_PORT,
                        Filesystem.getDeployDirectory() + "/" + DashboardConstants.DASHBOARD_PATH);
                if (Robot.isSimulation()) {
                    java.awt.Desktop.getDesktop()
                            .browse(java.net.URI.create("http://127.0.0.1:" + DashboardConstants.DASHBOARD_PORT));
                }
            } catch (Exception e) {
                System.out.println("Dashboard Webserver failed to start: " + e.getMessage());
            }
        }
        // host deploy directory webserver
        if (DashboardConstants.DEPLOY_SERVER_ENABLED) {
            System.out.println("Starting Deploy Webserver");
            System.out.println("Deploy directory: "
                    + Filesystem.getDeployDirectory()
                    + "/"
                    + DashboardConstants.DEPLOY_SERVER_PATH);
            try {
                WebServer.start(
                        DashboardConstants.DEPLOY_SERVER_PORT,
                        Filesystem.getDeployDirectory() + "/" + DashboardConstants.DEPLOY_SERVER_PATH);
                if (Robot.isSimulation()) {
                    java.awt.Desktop.getDesktop()
                            .browse(java.net.URI.create("http://127.0.0.1:" + DashboardConstants.DEPLOY_SERVER_PORT));
                }
            } catch (Exception e) {
                System.out.println("Deploy Webserver failed to start: " + e.getMessage());
            }
        }

        // Check for valid swerve config
        // var modules =
        //     new SwerveModuleConstants[] {
        //       TunerConstants.FrontLeft,
        //       TunerConstants.FrontRight,
        //       TunerConstants.BackLeft,
        //       TunerConstants.BackRight
        //     };
        // for (var constants : modules) {
        //   if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
        //       || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        //     throw new RuntimeException(
        //         "You are using an unsupported swerve configuration, which this template does not
        // support without manual customization. The 2025 release of Phoenix supports some swerve
        // configurations which were not available during 2025 beta testing, preventing any development
        // and support from the AdvantageKit developers.");
        //   }
        // }

        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Optionally switch the thread to high priority to improve loop
        // timing (see the template project documentation for details)
        // Threads.setCurrentThreadPriority(true, 99);

        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled commands, running already-scheduled commands, removing
        // finished or interrupted commands, and running subsystem periodic() methods.
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Return to non-RT thread priority (do not modify the first argument)
        // Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}
}
