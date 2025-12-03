package frc.lib.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FlydigiApex4 extends CommandXboxController {

    public FlydigiApex4(int pDriverStationPort) {
        super(pDriverStationPort);
    }

    public Trigger selectButton() {
        return super.button(7);
    }

    public Trigger startButton() {
        return super.button(8);
    }
}
