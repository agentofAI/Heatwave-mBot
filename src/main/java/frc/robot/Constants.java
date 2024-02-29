/*  VC: From our working chassis robot using the Kraken X60 flipped and gear ratios. 
     The constants were updated and caliberated in this file to make it work. Changes / tuning might be needed again.
     Changes to 
        chosenModule using WCP.SwerveXFlipped.KrakenX60(

*/


package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X2_10);
        /* VC: This must be tuned to specific robot */

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.375); /* VC: To Do VC: Confirm this. This must be tuned to specific robot */
        public static final double wheelBase = Units.inchesToMeters(23.375); /* VC: To Do: Confirm this. This must be tuned to specific robot */
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         /* VC: This is getting Kinematics created by finding robot center ?  */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        /* VC: Tuning needed ?  */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        /* VC: Tuning needed ?  */
        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        /* VC: PID gains need Tuning needed ?  */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        /* VC: PID gains need Tuning needed ?  */
        public static final double driveKP = 0.12; /* VC: To Do VC: This must be tuned to specific robot */
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        /* VC: To Do: Drive Motor characterization for speed ? , velocity and acceleration. This is empirically tested for the robots using SysID.
         * In our case, we will be doing trial error, or refer to Mercs for guidance
        */
        public static final double driveKS = 0.32; /* VC: To Do: This must be tuned to specific robot */
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; /* VC: To Do VC: This must be tuned to specific robot ? */
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; /* VC: To Do VC : This must be tuned to specific robot */

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { /* VC: To Do VC: Reconfirm IDs and Module locations. This must be tuned to specific robot */
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 23;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-91.5+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { /* VC: To Do: Reconfirm IDs and Module locations. This must be tuned to specific robot */
            public static final int driveMotorID = 35;
            public static final int angleMotorID = 34;
            public static final int canCoderID = 22;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-119.6);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { /* VC: To Do: Reconfirm IDs and Module locations. This must be tuned to specific robot */
            public static final int driveMotorID = 32;
            public static final int angleMotorID = 33;
            public static final int canCoderID = 21;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-53.3);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { /* VC: To Do: Reconfirm IDs and Module locations. This must be tuned to specific robot */
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 30;
            public static final int canCoderID = 20;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(25.3+180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { /* VC: To Do: Confirm / Tune values. The below constants are used in the example auto, and must be tuned to specific robot */
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}

