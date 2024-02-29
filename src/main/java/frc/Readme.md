Main - OK 
	Robot - OK 
        Init and Periodic methods 
            for container, teleop and auto
            init calls RobotContainer

        robotPeriodic
            responsible for calling subsystem's periodic methods
        autonomousInit
            calls getAutonomousCommand from RobotContainer

		RobotContainer - OK (Refactored, do 2nd pass if time allows)
            Initialize Subsystem
            JoyStick and Button configuration
            getButtonBinding - button to command mapping for teleop 
            setDefaultCommand to TeleOpSwerve
			Autonomous Command - GetAutonomousCommand - 
                call exampleAuto in getAutonomousCommand

SwerveModule - To Check

autos
    exampleAuto OK (Refactored, do 2nd pass if time allows)
                    Trajectory creation
                    Trajectory following - using SwerveControllerCommand
                    use SwerveDrive
subsystems
    SwerveDrive - To Check
    VC - To add Shooter subsystem 
commands
    TeleopSwerve - To Check
    VC - To add Shooter subsystem commands