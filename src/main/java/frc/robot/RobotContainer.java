
/*  VC: Copy over from our working chassis robot. First step is to test this.
    This will need to be updated for final code - aligned with template/example best practices for declaration and structuring of code. 
*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import java.util.List;

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

    
    // VC: Update to XBox Controller ? No. We use Logitech, assumedly in xInputmode
    /* Controllers */
    private final Joystick driver = new Joystick(0);

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
    // VC: Which button toggles speed ? Check the SwerveDrive code too for speed toggling. 
    private final JoystickButton toggleSpeed = new JoystickButton(driver, 6);

    // VC: Update based on sub-system organization
    // The robot's subsystems
    //  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final SwerveDrive s_swerveDrive = new SwerveDrive();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        
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

        // VC: We might not need this twice
        // Configure the button bindings
        //configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_swerveDrive.zeroHeading()));
        toggleSpeed.onTrue(new InstantCommand(() -> s_swerveDrive.toggleSpeed()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_swerveDrive);
    }
}
