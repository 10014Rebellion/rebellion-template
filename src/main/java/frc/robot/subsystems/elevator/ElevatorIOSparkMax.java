package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorController;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorMotor;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkBase mElevatorSpark;
    private final RelativeEncoder mElevatorEncoder;
    private final ProfiledPIDController mElevatorController;

    public ElevatorIOSparkMax() {
        this.mElevatorSpark = new SparkMax(ElevatorMotor.kMotorID, ElevatorMotor.kMotorType);
        this.mElevatorEncoder = mElevatorSpark.getEncoder();
        this.mElevatorController = new ProfiledPIDController(
                ElevatorController.kP,
                ElevatorController.kI,
                ElevatorController.kD,
                new Constraints(ElevatorController.kMaxVelocity, ElevatorController.kMaxAcceleration));
    }

    // private FunctionalCommand setPIDCmd() {
    //   return new FunctionalCommand(
    //     () -> {

    //     },
    //     () -> {

    //     },
    //     (interrupted) -> {},
    //     () -> false,
    //     this
    //   );
    // }

    @Override
    public void setElevatorOpenLoop(double output) {
        mElevatorSpark.set(MathUtil.clamp(output, -1, 1));
    }

    @Override
    public void setElevatorPosition(Rotation2d rotation) {}
}
