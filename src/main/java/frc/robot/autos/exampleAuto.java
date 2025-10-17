/*  VC: Copy over from our working chassis robot. First step is to test this.
    This will need to be updated for final code - aligned with template/example best practices for declaration and structuring of code. 
    We will be using PathWeaver - So update this accordingly based on trajectory generation, import and follow
*/

package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(SwerveDrive s_swerveDrive){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    // VC: Check Constants.Swerve for the speed limits.
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        /* VC: This is sample trajectory, to be updated baesd on path weaver. 
            We will concatenate 2 trajectories - start to note taking, note to speaker
        */ 
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //VC: So we (should) have and are using the SwerveController functionality 
        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_swerveDrive::getPose, // Functional interface to feed supplier
                Constants.Swerve.swerveKinematics, // Swerve.swerveKinematics = DriveConstants.kDriveKinematics
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_swerveDrive::setModuleStates,
                s_swerveDrive);

        // VC: Reset odometry to the initial pose of the trajectory, run path following
        // command, then stop at the end.
        /*    
        return Commands.sequence(
        new InstantCommand(() -> s_swerveDrive.setPose(exampleTrajectory.getInitialPose())),            
        //new InstantCommand(() -> s_swerveDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand //,
        //new InstantCommand(() -> s_swerveDrive.drive(0, 0, 0, false))
        );
        */

        //Covered above, which is elegant and more explanatory with explicit stop 
        addCommands(
            new InstantCommand(() -> s_swerveDrive.setPose(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
        



    }
}