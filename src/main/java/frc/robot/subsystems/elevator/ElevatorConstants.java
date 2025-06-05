package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
  public static final SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

  public class ElevatorMotor {
    public static int kMotorID = 41;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static int kCurrentLimitAmp = 60;
    public static boolean kMotorInverted = false;
  }

  public class ElevatorEncoder {
    public static boolean kEncoderInverted = false;
    public static double kPositionConversionFactor = 1;
    public static double kVelocityConversionFactor = 1;
  }

  public class ElevatorController {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double kG = 0;
    public static double kS = 0;
    public static double kV = 0;
    public static double kA = 0;

    public static double kMaxVelocity = 0;
    public static double kMaxAcceleration = 0;

  }

  public enum ElevatorStates {
    NONE,
    L1,
    L2,
    L3,
    L4,
    BARGE,
    PROCESSOR
  }

  static {
    kElevatorConfig
      .idleMode(ElevatorMotor.kIdleMode)
      .smartCurrentLimit(ElevatorMotor.kCurrentLimitAmp)
      .inverted(ElevatorMotor.kMotorInverted)
      .voltageCompensation(12.0);

    kElevatorConfig.encoder
      .inverted(ElevatorEncoder.kEncoderInverted)
      .positionConversionFactor(1)
      .velocityConversionFactor(1)
      .uvwMeasurementPeriod(10)
      .uvwAverageDepth(2);
  }
}
