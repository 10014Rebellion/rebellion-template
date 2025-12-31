// REBELLION 10014

package frc.robot.systems.drive.modules;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.systems.drive.DriveConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.systems.drive.DriveConstants;
import frc.robot.systems.drive.DriveConstants.ModuleHardwareConfig;

public class ModuleIOKraken implements ModuleIO {
    private TalonFX mDriveMotor;
    private VelocityTorqueCurrentFOC mDriveControl = new VelocityTorqueCurrentFOC(0.0);
    private VoltageOut mDriveVoltageControl = new VoltageOut(0.0);
    private double mDriveAppliedVolts = 0.0;

    private StatusSignal<Angle> mDrivePositionM;
    private StatusSignal<AngularVelocity> mDriveVelocityMPS;
    private StatusSignal<Voltage> mDriveVoltage;
    private StatusSignal<Current> mDriveSupplyCurrent;
    private StatusSignal<Current> mDriveStatorCurrent;
    private StatusSignal<Current> mDriveTorqueCurrent;
    private StatusSignal<Temperature> mDriveTempCelsius;
    private StatusSignal<AngularAcceleration> mDriveAccelerationMPSS;

    private TalonFX mAzimuthMotor;
    private PositionDutyCycle mAzimuthPositionControl = new PositionDutyCycle(0.0);
    private VoltageOut mAzimuthVoltageControl = new VoltageOut(0.0);
    private double mAzimuthAppliedVolts = 0.0;

    private StatusSignal<Angle> mAzimuthPosition;
    private StatusSignal<AngularVelocity> mAzimuthVelocity;
    private StatusSignal<Voltage> mAzimuthVoltage;
    private StatusSignal<Current> mAzimuthStatorCurrent;
    private StatusSignal<Current> mAzimuthSupplyCurrent;
    private StatusSignal<Temperature> mAzimuthTemp;

    private CANcoder mAbsoluteEncoder;
    private StatusSignal<Angle> mAbsolutePositionSignal;
    private Rotation2d mAbsoluteEncoderOffset;

    public ModuleIOKraken(ModuleHardwareConfig config) {
        mDriveMotor = new TalonFX(config.driveID(), DriveConstants.kDriveCANBusName);
        var driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = kDriveStatorAmpLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = kDriveSupplyAmpLimit;

        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;

        driveConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        driveConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = kDriveMotorGearing / kWheelCircumferenceMeters;

        driveConfig.Slot0.kP = kModuleControllerConfigs.driveController().getP();
        driveConfig.Slot0.kI = kModuleControllerConfigs.driveController().getI();
        driveConfig.Slot0.kD = kModuleControllerConfigs.driveController().getD();
        mDrivePositionM = mDriveMotor.getPosition();
        mDriveVelocityMPS = mDriveMotor.getVelocity();
        mDriveVoltage = mDriveMotor.getMotorVoltage();
        mDriveSupplyCurrent = mDriveMotor.getSupplyCurrent();
        mDriveStatorCurrent = mDriveMotor.getStatorCurrent();
        mDriveTorqueCurrent = mDriveMotor.getTorqueCurrent();
        mDriveTempCelsius = mDriveMotor.getDeviceTemp();
        mDriveAccelerationMPSS = mDriveMotor.getAcceleration();

        mDriveMotor.getConfigurator().apply(driveConfig);

        /* CANCODER INSTANTIATION AND CONFIGURATION */
        mAbsoluteEncoderOffset = Rotation2d.fromRotations(config.offset());
        
        mAbsoluteEncoder = new CANcoder(config.encoderID(), DriveConstants.kDriveCANBusName);
        
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));
        encoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(config.offset()));
        mAbsoluteEncoder.getConfigurator().apply(encoderConfig);
        
        mAbsolutePositionSignal = mAbsoluteEncoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, mAbsolutePositionSignal);
        mAbsoluteEncoder.optimizeBusUtilization();

        /* AZIMUTH INSTANTIATION AND CONFIGURATION */
        mAzimuthMotor = new TalonFX(config.azimuthID(), DriveConstants.kDriveCANBusName);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();

        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.CurrentLimits.StatorCurrentLimit = kAzimuthStatorAmpLimit;
        turnConfig.Voltage.PeakForwardVoltage = kPeakVoltage;
        turnConfig.Voltage.PeakReverseVoltage = -kPeakVoltage;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turnConfig.MotorOutput.Inverted = kTurnMotorInvert ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        turnConfig.Feedback.FeedbackRemoteSensorID = mAbsoluteEncoder.getDeviceID();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnConfig.Feedback.SensorToMechanismRatio = kCANCoderToMechanismRatio;
        turnConfig.Feedback.RotorToSensorRatio = kAzimuthMotorGearing;
        turnConfig.Slot0.kP = kModuleControllerConfigs.azimuthController().getP();
        turnConfig.Slot0.kD = kModuleControllerConfigs.azimuthController().getD();
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Configured but FOC not used on azimuth, just drive motors */
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = kAzimuthFOCAmpLimit;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -kAzimuthFOCAmpLimit;

        mAzimuthMotor.getConfigurator().apply(turnConfig);

        mAzimuthPosition = mAzimuthMotor.getPosition();
        mAzimuthVelocity = mAzimuthMotor.getVelocity();
        mAzimuthVoltage = mAzimuthMotor.getMotorVoltage();
        mAzimuthStatorCurrent = mAzimuthMotor.getStatorCurrent();
        mAzimuthSupplyCurrent = mAzimuthMotor.getSupplyCurrent();
        mAzimuthTemp = mAzimuthMotor.getDeviceTemp();

        resetAzimuthEncoder();
    }

    @Override
    public void updateInputs(ModuleInputs inputs) {
        inputs.iIsDriveConnected = BaseStatusSignal.refreshAll(
                        mDriveVelocityMPS,
                        mDrivePositionM,
                        mDriveVoltage,
                        mDriveSupplyCurrent,
                        mDriveStatorCurrent,
                        mDriveTorqueCurrent,
                        mDriveTempCelsius)
                .isOK();
        inputs.iDrivePositionM = (mDrivePositionM.getValueAsDouble());
        inputs.iDriveVelocityMPS = (mDriveVelocityMPS.getValueAsDouble());
        inputs.iDriveAppliedVolts = mDriveAppliedVolts;
        inputs.iDriveMotorVolts = mDriveVoltage.getValueAsDouble();
        inputs.iDriveSupplyCurrentAmps = mDriveSupplyCurrent.getValueAsDouble();
        inputs.iDriveStatorCurrentAmps = mDriveStatorCurrent.getValueAsDouble();
        inputs.iDriveTorqueCurrentAmps = mDriveTorqueCurrent.getValueAsDouble();
        inputs.iDriveTemperatureCelsius = mDriveTempCelsius.getValueAsDouble();
        inputs.iDriveAccelerationMPSS = mDriveAccelerationMPSS.getValueAsDouble();

        inputs.iIsAzimuthConnected = BaseStatusSignal.refreshAll(
                        mAzimuthVelocity,
                        mAzimuthVoltage,
                        mAzimuthStatorCurrent,
                        mAzimuthSupplyCurrent,
                        mAzimuthTemp,
                        mAzimuthPosition)
                .isOK();
        inputs.iAzimuthPosition = Rotation2d.fromRotations(mAzimuthPosition.getValueAsDouble());
        inputs.iAzimuthVelocity = Rotation2d.fromRotations(mAzimuthVelocity.getValueAsDouble());
        inputs.iAzimuthAppliedVolts = mAzimuthAppliedVolts;
        inputs.iAzimuthMotorVolts = mAzimuthVoltage.getValueAsDouble();
        inputs.iAzimuthStatorCurrentAmps = mAzimuthStatorCurrent.getValueAsDouble();
        inputs.iAzimuthSupplyCurrentAmps = mAzimuthSupplyCurrent.getValueAsDouble();
        inputs.iAzimuthTemperatureCelsius = mAzimuthTemp.getValueAsDouble();

        inputs.iIsCancoderConnected = BaseStatusSignal.refreshAll(mAbsolutePositionSignal).isOK();
        inputs.iAzimuthAbsolutePosition = Rotation2d.fromRotations(mAbsolutePositionSignal.getValueAsDouble());
    }

    /////////// DRIVE MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setDriveVolts(double volts) {
        mDriveMotor.setControl(mDriveVoltageControl.withOutput(volts));
    }

    @Override
    public void setDriveAmperage(double amps) {
        mDriveMotor.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setDriveVelocity(double velocityMPS, double feedforward) {
        mDriveMotor.setControl(mDriveControl.withVelocity(velocityMPS).withFeedForward(feedforward));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        mDriveMotor.getConfigurator().apply(slotConfig);
    }

    /////////// CANCODER METHODS \\\\\\\\\\\
    @Override
    public void resetAzimuthEncoder() {
        mAzimuthMotor.setPosition(Rotation2d.fromRotations(mAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()).getRotations());
    }

    /////////// AZIMUTH MOTOR METHODS \\\\\\\\\\\
    @Override
    public void setAzimuthVolts(double volts) {
        /* Sets azimuth voltage inbetween kPeakVoltage and -kPeakVoltage */
        mAzimuthMotor.setControl(mAzimuthVoltageControl.withOutput(volts));
    }

    @Override
    public void setAzimuthPosition(Rotation2d rotation, double feedforward) {
        /* Uses voltage PID with a arbitrary FF on the with Slot 0 gains */
        mAzimuthMotor.setControl(mAzimuthPositionControl
                .withPosition(rotation.getRotations())
                .withFeedForward(feedforward)
                .withSlot(0));
    }

    /* Sets azimuth PID values on slot 0, used for tunable numbers */
    @Override
    public void setAzimuthPID(double kP, double kI, double kD) {
        var slotConfig = new Slot0Configs();
        slotConfig.kP = kP;
        slotConfig.kI = kI;
        slotConfig.kD = kD;
        mAzimuthMotor.getConfigurator().apply(slotConfig);
    }
}
