package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {

    XboxController controller;
    Arm arm;

    public MoveArm(XboxController controller, Arm arm) {
        this.controller = controller;
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.moveWrist(controller.getRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
