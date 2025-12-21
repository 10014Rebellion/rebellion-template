package frc.robot.bindings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.controllers.HolonomicController.ConstraintType;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final FlydigiApex4 mDriverController = new FlydigiApex4(BindingsConstants.kDriverControllerPort);

    public ButtonBindings(Drive pDriveSS) {
        this.mDriveSS = pDriveSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.setToTeleop());
    }

    public void initDriverButtonBindings() {
        mDriveSS.acceptJoystickInputs(
            () -> - mDriverController.getLeftY(), 
            () -> - mDriverController.getLeftX(), 
            () -> - mDriverController.getRightX(), 
            () -> mDriverController.getPOVAngle()
        );

        mDriverController.a()
            .onTrue(mDriveSS.setToGenericLineAlign(() -> 
                new Pose2d(3, 3, Rotation2d.fromRadians(Math.PI / 4.0)), 
                () -> Rotation2d.fromRadians(Math.PI / 4.0)))
            .onFalse(mDriveSS.setToTeleop());

        mDriverController.b()
            .onTrue(mDriveSS.setToGenericAutoAlign(() -> 
                new Pose2d(3, 3, Rotation2d.fromRadians(Math.PI / 4.0)), 
                ConstraintType.LINEAR))
            .onFalse(mDriveSS.setToTeleop());
    }
}
