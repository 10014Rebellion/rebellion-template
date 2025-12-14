// REBELLION 10014

package frc.robot.systems.drive.modules;

import static frc.robot.systems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleIOSim implements ModuleIO {
    private DCMotorSim mDriveMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, kDriveMotorGearing),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);
    private DCMotorSim mAzimuthMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.025, 52),
            DCMotor.getKrakenX60Foc(1),
            0.0,
            0.0);

    private double mDriveAppliedVolts = 0.0;
    private double mAzimuthAppliedVolts = 0.0;

    private PIDController mDrivePID = kModuleControllerConfigs.driveController();

    private PIDController mAzimuthPID = kModuleControllerConfigs.azimuthController();

    public ModuleIOSim() {
        mAzimuthPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        mDriveMotor.update(0.02);
        mAzimuthMotor.update(0.02);

        inputs.iDrivePositionM = mDriveMotor.getAngularPositionRotations() * kWheelCircumferenceMeters;
        inputs.iDriveVelocityMPS = (mDriveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters) / 60.0;
        inputs.iDriveAppliedVolts = mDriveAppliedVolts;
        inputs.iDriveStatorCurrentAmps = Math.abs(mDriveMotor.getCurrentDrawAmps());
        inputs.iDriveTemperatureCelsius = 0.0;
        inputs.iAzimuthAppliedVolts = mAzimuthAppliedVolts;
        inputs.iAzimuthMotorVolts = mAzimuthAppliedVolts;

        inputs.iAzimuthAbsolutePosition = new Rotation2d(mAzimuthMotor.getAngularPositionRad());
        inputs.iAzimuthPosition = new Rotation2d(mAzimuthMotor.getAngularPositionRad());
        inputs.iAzimuthVelocity = Rotation2d.fromRadians(mAzimuthMotor.getAngularVelocityRadPerSec());
        inputs.iAzimuthStatorCurrentAmps = Math.abs(mAzimuthMotor.getCurrentDrawAmps());
        inputs.iAzimuthTemperatureCelsius = 0.0;
        inputs.iAzimuthAppliedVolts = mAzimuthAppliedVolts;
        inputs.iAzimuthMotorVolts = mAzimuthAppliedVolts;
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        /* sets drive voltage between -kPeakVoltage and kPeakVoltage */
        mDriveAppliedVolts = MathUtil.clamp(volts, -kPeakVoltage, kPeakVoltage);
        mDriveMotor.setInputVoltage(mDriveAppliedVolts);
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        /* Sets drive velocity using PID */
        setDriveVolts(
                mDrivePID.calculate(mDriveMotor.getAngularVelocityRPM() * kWheelCircumferenceMeters / 60, velocityMPS)
                        + feedforward);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        /* Sets drive velocity PID */
        mDrivePID.setPID(kP, kI, kD);
    }

    @Override
    public void resetAzimuthEncoder() {
        /* No code is needed, sim doesn't need an implementation of this */
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        /* sets azimuth voltage between -kPeakVoltage and kPeakVoltage */
        mAzimuthAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        mAzimuthMotor.setInputVoltage(mAzimuthAppliedVolts);
    }

    @Override
    public void setAzimuthPosition(Rotation2d position, double feedforward) {
        /* Sets azimuth position using PID */
        setAzimuthVolts(
                mAzimuthPID.calculate(mAzimuthMotor.getAngularPositionRad(), position.getRadians()) + feedforward);
    }

    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        /* Sets azimuth position PID */
        mAzimuthPID.setPID(kP, kI, kD);
    }
}
