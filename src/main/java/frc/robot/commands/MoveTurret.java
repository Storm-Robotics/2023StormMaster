package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class MoveTurret extends CommandBase {

    XboxController controller;
    Turret turret;

    public MoveTurret(XboxController controller, Turret turret) {

        this.controller = controller;
        this.turret = turret;

        addRequirements(turret);

    }

    @Override
    public void execute() {

        turret.move(controller.getLeftX());

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
