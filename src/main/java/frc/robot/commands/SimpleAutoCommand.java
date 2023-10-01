package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SimpleAutoCommand extends CommandBase {

    private Swerve swerve;
    private Timer timer;

    public SimpleAutoCommand(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        
        while(timer.get() < 4) {
            swerve.drive(new Translation2d(0.4, 0), 0, false, true);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
