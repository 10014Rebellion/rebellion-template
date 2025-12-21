// REBELLION 10014

package frc.robot.systems.drive.modules;

import static frc.robot.systems.drive.DriveConstants.kModuleControllerConfigs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.telemetry.Telemetry;
import frc.lib.tuning.LoggedTunableNumber;
import frc.robot.systems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
    public static final LoggedTunableNumber driveP = new LoggedTunableNumber(
            "Module/Drive/kP", kModuleControllerConfigs.driveController().getP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber(
            "Module/Drive/kD", kModuleControllerConfigs.driveController().getD());
    public static final LoggedTunableNumber driveS = new LoggedTunableNumber(
            "Module/Drive/kS", kModuleControllerConfigs.driveFF().getKs());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber(
            "Module/Drive/kV", kModuleControllerConfigs.driveFF().getKv());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber(
            "Module/Drive/kA", kModuleControllerConfigs.driveFF().getKa());

    public static final LoggedTunableNumber turnP = new LoggedTunableNumber(
            "Module/AzimuthP", kModuleControllerConfigs.azimuthController().getP());
    public static final LoggedTunableNumber turnD = new LoggedTunableNumber(
            "Module/AzimuthD", kModuleControllerConfigs.azimuthController().getD());
    public static final LoggedTunableNumber turnS = new LoggedTunableNumber(
            "Module/AzimuthS", kModuleControllerConfigs.azimuthFF().getKs());

    private ModuleIO io;
    private ModuleInputsAutoLogged inputs = new ModuleInputsAutoLogged();

    private final String kModuleName;

    /* Drive Control */
    private Double velocitySetpointMPS = null;
    // driveA ACTS AS A FUDGE FACTOR, NOT ACTUAL CONSTANT FOR AMPERAGE TUNING
    private Double amperageFeedforward = null;
    private SimpleMotorFeedforward driveFF = DriveConstants.kModuleControllerConfigs.driveFF();

    /* Azimuth Control */
    private Rotation2d azimuthSetpointAngle = null;
    private SimpleMotorFeedforward azimuthFF = DriveConstants.kModuleControllerConfigs.azimuthFF();

    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    public Module(String key, ModuleIO io) {
        this.io = io;
        kModuleName = "Module/" + key;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + kModuleName, inputs);

        currentState = new SwerveModuleState(inputs.iDriveVelocityMPS, inputs.iAzimuthPosition);
        currentPosition = new SwerveModulePosition(inputs.iDrivePositionM, inputs.iAzimuthPosition);

        // Runs drive PID
        // if Amperage is null no feedforward is used
        // Amperage is the FOC feedforward
        if (velocitySetpointMPS != null) {
            // Telemetry.log("Drive/"+kModuleName+"/velocitySepointMPS", velocitySetpointMPS);
            if (amperageFeedforward != null) {
                double ffOutput = driveFF.calculateWithVelocities(velocitySetpointMPS, amperageFeedforward);

                Telemetry.log("Drive/" + kModuleName + "/AmperageFeedforward", amperageFeedforward);
                Telemetry.log("Drive/" + kModuleName + "/ffOutput", ffOutput);

                io.setDriveVelocity(velocitySetpointMPS, ffOutput);
            } else {
                io.setDriveVelocity(velocitySetpointMPS, 0.0);
            }
        }

        // Runs azimuth PID
        if (azimuthSetpointAngle != null) {
            double ffOutput = azimuthFF.getKs() * Math.signum(inputs.iAzimuthVelocity.getDegrees());
            Telemetry.log("Drive/" + kModuleName + "/SimpleFeedforward", ffOutput);
            io.setAzimuthPosition(azimuthSetpointAngle, ffOutput);
        }

        if (DriverStation.isDisabled()) stop();

        // Updates PID values for drive and azimuth
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    io.setDrivePID(driveP.get(), 0.0, driveD.get());
                },
                driveP,
                driveD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    driveFF = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
                },
                driveS,
                driveV,
                driveA);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    io.setAzimuthPID(turnP.get(), 0.0, turnD.get());
                },
                turnP,
                turnD);

        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> {
                    azimuthFF = new SimpleMotorFeedforward(turnS.get(), 0.0, 0.0);
                },
                turnS);
    }

    /* Sets the desired setpoint of the module with FF. Disables the all FF include velocity FF
     * @param state the desired velocity and rotation of the module
     */
    public SwerveModuleState setDesiredState(SwerveModuleState state) {
        setDesiredStateWithAmpFF(state, null);
        return getDesiredState();
    }

    /* Sets the desired setpoint of the module with FF
     * @param state the desired velocity and rotation of the module
     * @param ampFeedforward The amperage added to the PID from FF, also enables the PID
     */
    public SwerveModuleState setDesiredStateWithAmpFF(SwerveModuleState state, Double ampFeedforward) {
        setAmpFeedforward(ampFeedforward);
        setDesiredVelocity(state.speedMetersPerSecond);
        setDesiredRotation(state.angle);
        return getDesiredState();
    }

    /* Runs characterization of by setting motor drive voltage and rotates the module forward
     * With no closed loop
     * @param inputVolts -12  to 12
     */
    public void runCharacterization(double inputVolts) {
        runCharacterization(inputVolts, new Rotation2d());
    }

    /* Runs characterization of by setting motor drive voltage and the rotation the module at a specific rotation
     * With no closed loop
     * @param inputVolts -12  to 12
     * @param azimuthRotation Rotation at which the robot is running characterization voltage at
     */
    public void runCharacterization(double inputVolts, Rotation2d azimuthRotation) {
        setDesiredRotation(azimuthRotation);
        setDesiredVelocity(null);
        setDriveVoltage(inputVolts);
    }

    /* Sets the velocity of the module
     * @param velocitySetpoint the velocity setpoint
     */
    public void setDesiredVelocity(Double velocitySetpoint) {
        velocitySetpointMPS = velocitySetpoint;
    }

    /* Sets the amperage Feedforward
     * @pararm Rotation2d angleSetpoint
     */
    public void setAmpFeedforward(Double amperage) {
        this.amperageFeedforward = amperage;
    }

    /* Sets azimuth rotation goal
     * @pararm Rotation2d angleSetpoint
     */
    public void setDesiredRotation(Rotation2d angleSetpoint) {
        azimuthSetpointAngle = angleSetpoint;
    }

    /* Sets drive motor's voltage
     * @param driveVolts: -kPeakVoltage to PeakVoltage volts
     */
    public void setDriveVoltage(double driveVolts) {
        io.setDriveVolts(driveVolts);
    }

    /* Sets drive motor's voltage
     * @param driveVolts: -kDriveFOCAmpLimit to kDriveFOCAmpLimit volts
     */
    public void setDriveAmperage(double amps) {
        io.setDriveAmperage(amps);
    }

    /* Sets drive motor's voltage
     * @param azimuthVolts: -kPeakVoltage to PeakVoltage volts
     */
    public void setAzimuthVoltage(double azimuthVolts) {
        io.setAzimuthVolts(azimuthVolts);
    }

    /* Stops modules by setting voltage to zero */
    public void stop() {
        setDriveVoltage(0.0);
        setAzimuthVoltage(0.0);
    }

    /* Gets the setpoint state of the module(speed and rotation) */
    public SwerveModuleState getDesiredState() {
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    /* Gets the physical state of the module(speed and rotation) */
    public SwerveModuleState getCurrentState() {
        return currentState;
    }

    /* Gets the physical position of the module(position and rotation) */
    public SwerveModulePosition getCurrentPosition() {
        return currentPosition;
    }

    /* All logged hardware data in the module */
    public ModuleInputsAutoLogged getInputs() {
        return inputs;
    }

    /* Resets azimuth encoder from CANCoder */
    public void resetAzimuthEncoder() {
        io.resetAzimuthEncoder();
    }
}
