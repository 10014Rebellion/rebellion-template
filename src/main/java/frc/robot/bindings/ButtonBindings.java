// REBELLION 10014

package frc.robot.bindings;

import frc.lib.controllers.FlydigiApex4;
import frc.robot.systems.drive.Drive;

public class ButtonBindings {
    private final Drive mDriveSS;
    private final FlydigiApex4 mDriverController = new FlydigiApex4(BindingsConstants.kDriverControllerPort);

    public ButtonBindings(Drive pDriveSS) {
        this.mDriveSS = pDriveSS;
        this.mDriveSS.setDefaultCommand(mDriveSS.setToTeleop());
    }

    public void initDriverButtonBindings() {
        mDriveSS.acceptJoystickInputs(
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> -mDriverController.getRightX(),
                () -> mDriverController.getPOVAngle());
    }
}
