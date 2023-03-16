package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final XboxController controller = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Controller Buttons */
    private final JoystickButton elevatorUp = new JoystickButton(controller, XboxController.Button.kY.value);
    private final JoystickButton elevatorDown = new JoystickButton(controller, XboxController.Button.kA.value);
    private final JoystickButton elevatorOut = new JoystickButton(controller, XboxController.Button.kB.value);
    private final JoystickButton elevatorIn = new JoystickButton(controller, XboxController.Button.kX.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Turret turret = new Turret();
    private final VerticalElevator verticalElevator = new VerticalElevator();
    private final HorizontalElevator horizontalElevator = new HorizontalElevator();
    private final Arm arm = new Arm();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        setDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        /* Controller Buttons */
        elevatorUp.onTrue(new InstantCommand(() -> verticalElevator.moveUp()));
        elevatorDown.onTrue(new InstantCommand(() -> verticalElevator.moveDown()));
        elevatorIn.onTrue(new InstantCommand(() -> horizontalElevator.moveIn()));
        elevatorOut.onTrue(new InstantCommand(() -> horizontalElevator.moveOut()));
    }

    private void setDefaultCommands() {
        turret.setDefaultCommand(new MoveTurret(controller, turret));
        arm.setDefaultCommand(new InstantCommand(() -> arm.moveWrist(controller.getRightY())));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }

    public Command configureAutonomous() {

        ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("ChargeStationAuto", new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose,
            s_Swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(0.5, 0, 0),
            new PIDConstants(0.5, 0, 0),
            s_Swerve::setModuleStates,
            eventMap,
            true,
            s_Swerve
        );

        return autoBuilder.fullAuto(pathGroup);

    }
}
