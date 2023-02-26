package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
    
    CANSparkMax master = new CANSparkMax(Constants.RobotComponents.turretMotorID, MotorType.kBrushless);
    RelativeEncoder encoder = master.getEncoder();
    SparkMaxPIDController pidController = master.getPIDController();

    public Turret() {

        encoder.setPosition(0);
        encoder.setPositionConversionFactor(Constants.RobotComponents.turretGearboxReduction);

        setPID();

    }

    private void setPID() {
        pidController.setP(Constants.RobotComponents.turretMotorPID[0]);
        pidController.setI(Constants.RobotComponents.turretMotorPID[1]);
        pidController.setD(Constants.RobotComponents.turretMotorPID[2]);
        pidController.setFF(Constants.RobotComponents.turretMotorPID[3]);
    }

    /**
     * Moves Turret to desired heading
     * @param position position in degrees
     * @return error
     */
    public double moveTo(double position) {

        double revolutions = (position/360) - encoder.getPosition();

        pidController.setReference(position/360, CANSparkMax.ControlType.kPosition);

        double error = (position/360 - encoder.getPosition()) * 360;
        SmartDashboard.putNumber("Turret Error", error);
        return error;

    }

    /**
     * Returns the heading of the Turret
     * @return Heading in degrees of Turret
     */
    public double getHeading() {
        return encoder.getPosition() * 360;
    }
    

}
