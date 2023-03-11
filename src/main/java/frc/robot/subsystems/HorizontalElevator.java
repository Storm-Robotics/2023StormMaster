package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HorizontalElevator extends SubsystemBase {

    private final VictorSPX master;

    public HorizontalElevator() {
        this.master = new VictorSPX(Constants.RobotComponents.horizontalMaster);
    }

    public void moveOut() {
        master.set(ControlMode.Velocity, 1 * Constants.RobotComponents.HORIZONTAL_ELEVATOR_SPEED_MULTIPLIER);
    }

    public void moveIn() {

        master.set(ControlMode.Velocity, -1 * Constants.RobotComponents.HORIZONTAL_ELEVATOR_SPEED_MULTIPLIER);

    }
    
}
