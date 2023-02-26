package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;

public class StaticTurret extends CommandBase {
    
    private Turret turret;
    private Swerve swerve;

    public StaticTurret(Turret turret, Swerve swerve) {
        this.turret = turret;
        this.swerve = swerve;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.moveTo(180-swerve.getYaw().getDegrees());
    }

    public boolean isFinished() {
        return Math.abs((180.0 - turret.getHeading())) < 5.0;
    }

}
