package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {

    CANSparkMax wrist;
    VictorSPX intake;
    RelativeEncoder encoder;

    SparkMaxPIDController pidController;

    double currentAngle;

    public Arm() {

        this.wrist = new CANSparkMax(Constants.RobotComponents.wrist, MotorType.kBrushless);
        this.intake = new VictorSPX(Constants.RobotComponents.intake);

        this.encoder = wrist.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

        this.pidController = wrist.getPIDController();
        pidController.setFeedbackDevice(encoder);

        pidController.setP(Constants.RobotComponents.wristMotorPID[0]);
        pidController.setI(Constants.RobotComponents.wristMotorPID[1]);
        pidController.setD(Constants.RobotComponents.wristMotorPID[2]);
        pidController.setFF(Constants.RobotComponents.wristMotorPID[3]);
        // pidController.setOutputRange(-1, 1); //TODO Fix this

    }

    public void intakeIn() {
        intake.set(ControlMode.Velocity, 1);
    }

    public void intakeOut() {
        intake.set(ControlMode.Velocity, -1);
    }

    public double moveWristTo(double heading) {

        if(heading > Constants.RobotComponents.MAX_ANGLE) {
            heading = Constants.RobotComponents.MAX_ANGLE;
        } else if(heading < Constants.RobotComponents.MIN_ANGLE) {
            heading = Constants.RobotComponents.MIN_ANGLE;
        }

        pidController.setReference(heading/360, CANSparkMax.ControlType.kSmartMotion);
        
        double error = (heading/360 - encoder.getPosition()) * 360;
        SmartDashboard.putNumber("Turret Error", error);
        return error;

    }

}