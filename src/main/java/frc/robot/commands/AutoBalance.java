package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {

    private double speed = 0.2;
    private double offBalancePositiveHalf = 7;
    public final Swerve swerve;

    public AutoBalance(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void execute() {

        if(swerve.getPitch() >= offBalancePositiveHalf) {
            swerve.drive(new Translation2d(speed, 0), 0, true, true);
        } else if (swerve.getPitch() < -offBalancePositiveHalf) {
            swerve.drive(new Translation2d(-speed, 0), 0, true, true);
        } else {
            swerve.drive(new Translation2d(0, 0), 0, true, true);
        }

        speed = swerve.getPitch() / 150;

    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, false, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getPitch()) < 3;
    }
    
}
