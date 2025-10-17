
/*  VC: Copy over from our working chassis robot. First step is to test this.
    This will need to be updated for final code - aligned with template/example best practices for declaration and structuring of code. 
*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


    private final SendableChooser<Command> autoChooser;

    // VC: Update to XBox Controller ? No. We use Logitech, assumedly in xInputmode
    /* Controllers */
    private final Joystick driver = new Joystick(Constants.OperatorConstants.kDriverControllerPort);
    //private final Joystick operator = new Joystick(Constants.OperatorConstants.kOperatorControllerPort);
    private final CommandXboxController operator = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // VC: The drive controls and buttons should probably move to buttonBindings.
    /* Drive Controls */
    // translate - Vertical, strafe - horizonal,  rotation - rotatoin 
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    // VC: How is zeroGyro used in SwerveDrive ? 
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    // VC: How is robotCentric used in SwerveDrive ? 
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    
    // VC: Which button toggle speed ? Check the SwerveDrive code too for speed toggling. 
    //private final JoystickButton toggleSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);    
    private final JoystickButton reduceSpeed = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton increaseSpeed = new JoystickButton(driver, XboxController.Button.kRightStick.value); 
    //private final JoystickButton increaseSpeed = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value); 
    
    /* Operator Buttons - To Add */
    //here



    // VC: Update based on sub-system organization
    // The robot's subsystems
    //  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final SwerveDrive s_swerveDrive = new SwerveDrive();    
    //private final CANDrivetrain m_drivetrain = new CANDrivetrain();
    private final CANLauncher m_launcher = new CANLauncher();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        

        // Register Named Commands for Auto Routine from PathPlanner
        NamedCommands.registerCommand("IntakeNote", m_launcher.getIntakeCommand());
        NamedCommands.registerCommand("LaunchNote", 
                                      new PrepareLaunch(m_launcher)
                                        .withTimeout(LauncherConstants.kLauncherDelay)
                                        .andThen(new LaunchNote(m_launcher))
                                        .handleInterrupt(() -> m_launcher.stop()));        

        // VC : Configure button bindings - pose, orientation, speed throttle, field orientation
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        s_swerveDrive.setDefaultCommand(
            // VC: The default command is TeleopSwerve. 
            // VC: Update the logic here or in Teleop as needed. 
            // When does it get executed ?  Baesd on execute method, which assumedly will be called when the command is instantiated ?
            // What does it do? Drives the bot - based on Joystick inputs multiplied by speed
            new TeleopSwerve(
                s_swerveDrive, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        /*Create an inline sequence to run when the operator presses and holds the A (green) button. Run the PrepareLaunch
        * command for 1 seconds and then run the LaunchNote command */
        operator
            .rightBumper()
            .whileTrue(
                new PrepareLaunch(m_launcher)
                    .withTimeout(LauncherConstants.kLauncherDelay)
                    .andThen(new LaunchNote(m_launcher))
                    .handleInterrupt(() -> m_launcher.stop()));

        // Set up a binding to run the intake command while the operator is pressing and holding the
        // left Bumper
        operator.leftBumper().whileTrue(m_launcher.getIntakeCommand());

    } //configure binding ends here 

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        // VC: Added after PathPlanner Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser built above
        SmartDashboard.putData("DS1 Auto", new PathPlannerAuto("Blue DS1 Auto.auto"));
        SmartDashboard.putData("DS2 Auto", new PathPlannerAuto("Blue DS2 Auto.auto"));
        SmartDashboard.putData("DS3 Auto", new PathPlannerAuto("Blue DS3 Auto.auto"));

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_swerveDrive.zeroHeading()));
        //toggleSpeed.onTrue(new InstantCommand(() -> s_swerveDrive.toggleSpeed()));
        reduceSpeed.onTrue(new InstantCommand(() -> s_swerveDrive.reduceSpeed()));
        increaseSpeed.onTrue(new InstantCommand(() -> s_swerveDrive.increaseSpeed()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return new exampleAuto(s_swerveDrive);

        //VC Get auto command from SmartDashboard. 
        return autoChooser.getSelected();

        //VC: Added for using Auto routines from the 3015 RangeRobotics PathPlanner util.
        //PathPlannerPath path = PathPlannerPath.fromPathFile("deploy/pathplanner/autos/blue_ds1_autoRoutine");
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);

    }
}
