package frc.utils.commands;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorCommand extends InstantCommand {
    public MotorCommand(MotorController controller, double speed, Subsystem... requirements) {
        super(() -> controller.set(speed), requirements);
    }

    public MotorCommand(MotorController controller, Subsystem... requirements) {
        super(() -> controller.stopMotor(), requirements);
    }
}
