// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.PicoLED;
// Constants
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
// Wildlib
import wildlib.utils.commands.DoubleEvent;
import wildlib.utils.ds.DoubleInput;

// Std
import java.util.List;
import java.util.function.DoubleSupplier;
// Navx-micro
import com.kauailabs.navx.frc.AHRS;
// Math
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// WPILib
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// Network Tables
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.networktables.NetworkTable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private static final Drive drive = Drive.getInstance();
    private static final Arm arm = Arm.getInstance();
    private static final Claw claw = Claw.getInstance();
    private static final PicoLED leds = PicoLED.getInstance();

    private static final AHRS navx = drive.getNavx();
    
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber targetOffsetHorizontal = limelight.getDoubleTopic("tx").subscribe(0.0);

    private double previousAccelX = 0.0;
    private double previousAccelY = 0.0;

    DoubleSupplier navxJerkX = () -> {
        double currentAccelX = navx.getWorldLinearAccelX();
        double jerkX = currentAccelX - previousAccelX;
        previousAccelX = currentAccelX;

        return jerkX;
    };

    DoubleSupplier navxJerkY = () -> {
        double currentAccelY = navx.getWorldLinearAccelY();
        double jerkY = currentAccelY - previousAccelY;
        previousAccelY = currentAccelY;

        return jerkY;
    };

    DoubleSupplier rightY = () -> {
        return -IOConstants.commandController.getRightY();
    };

    private static final double COLLISION_THRESHOLD = 0.5;

    private GenericSubscriber extension;
    private DoubleInput testInput;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        // We don't really need all this extra telemetry in a comp match.
        MatchType match = DriverStation.getMatchType();
        if (match == MatchType.None || match == MatchType.Practice) {
            initTelemetry(new SubsystemBase[] {
                drive,
                arm,
                claw,
            });
        }

        configureButtonBindings();
        configureDashboard();
        configureRumble();

        // Drive based on joystick input when no other command is running.
        drive.setDefaultCommand(
            new RunCommand(
                () -> {
                    double forward = MathUtil.applyDeadband(IOConstants.commandController.getLeftY(), IOConstants.translationDeadband);
                    double strafe = MathUtil.applyDeadband(IOConstants.commandController.getLeftX(), IOConstants.translationDeadband);
                    double rotation = MathUtil.applyDeadband(IOConstants.commandController.getRightX(), IOConstants.rotationDeadband);
                    double speed = IOConstants.commandController.getRightTriggerAxis();

                    drive.drive(
                        forward * speed * (IOConstants.xyInverted ? -1.0 : 1.0),
                        strafe * speed * (IOConstants.xyInverted ? -1.0 : 1.0),
                        rotation * speed * (IOConstants.rotInverted ? -1.0 : 1.0),
                        true, true
                    );
                }, drive
            )
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        IOConstants.commandController.x()
            .whileTrue(new RunCommand( // Use whileTrue instead of onTrue to prevent the default command from running
                () -> drive.crossWheels(), 
                drive
            ));

        IOConstants.commandController
            .leftTrigger()
            .whileTrue(new RunCommand(
                () -> {
                    double readOffset = targetOffsetHorizontal.get();
                    if (readOffset < 0.0) {
                        readOffset /= VisionConstants.leftFOV;
                    }  else {
                        readOffset /= VisionConstants.rightFOV;
                    }

                    drive.drive(0.0, 0.0, VisionConstants.turnPID.calculate(readOffset), false, true);
                }, drive
            ));

        IOConstants.commandController.rightBumper().onTrue(new InstantCommand(() -> System.out.println(navx.getAngle())));
        IOConstants.commandController.start().onTrue(new InstantCommand(() -> {
            System.out.println("Zeroing sensors");
            navx.zeroYaw();
            claw.zeroSensors();
        }));

        IOConstants.commandController.povRight()
            .onTrue(new InstantCommand(() -> claw.setContraction(0.5), claw))
            .onFalse(new InstantCommand(() -> claw.stopContraction(), claw));

        IOConstants.commandController.povLeft()
            .onTrue(new InstantCommand(() -> claw.setContraction(-0.5), claw))
            .onFalse(new InstantCommand(() -> claw.stopContraction(), claw));
            
            IOConstants.commandController.povUp()
            .onTrue(new InstantCommand(() -> claw.setRotation(-0.05), claw))
            .onFalse(new InstantCommand(() -> claw.holdRotation(), claw));

        IOConstants.commandController.povDown()
            .onTrue(new InstantCommand(() -> claw.setRotation(0.2), claw))
            .onFalse(new InstantCommand(() -> claw.holdRotation(), claw));

        new DoubleEvent(rightY, (y) -> y >= 0.05 || y <= -0.05)
            .castTo(Trigger::new)
            .onTrue(new InstantCommand(() -> arm.setExtension(rightY.getAsDouble() * 0.3), arm))
            .onFalse(new InstantCommand(() -> {
                arm.holdExtension();
                arm.zeroLimit();
            }, arm));

        IOConstants.commandController.y()
            .onTrue(new InstantCommand(() -> arm.setRotation(-0.2), arm))
            .onFalse(new InstantCommand(() -> arm.holdRotation(), arm));

        IOConstants.commandController.a()
            .onTrue(new InstantCommand(() -> arm.setRotation(0.2), arm))
            .onFalse(new InstantCommand(() -> arm.holdRotation(), arm));

        IOConstants.commandController.back()
            .onTrue(new RunCommand(() -> arm.setExtensionPosition(extension.getDouble(10.0))))
            .onFalse(new InstantCommand(() -> {
                arm.zeroLimit();
                arm.holdExtension();
            }));
    }

    private void configureDashboard() {
        SmartDashboard.putNumber("Extension", 10.0);
        extension = SmartDashboard.getEntry("Extension").getTopic().genericSubscribe();

        RunCommand cross = new RunCommand(() -> {
            drive.crossWheels();
        }, drive);

        SmartDashboard.putData("Cross Wheels", cross);

        testInput = new DoubleInput("Test Input", 10.0);
        System.out.println(testInput.get());
    }

    private void configureRumble() {
        new DoubleEvent(navxJerkX, (double jerk) -> Math.abs(jerk) > COLLISION_THRESHOLD)
            .castTo(Trigger::new)
            .onTrue(new InstantCommand(() -> IOConstants.controller.setRumble(RumbleType.kBothRumble, 1.0)));

        new DoubleEvent(navxJerkY, (double jerk) -> Math.abs(jerk) > COLLISION_THRESHOLD)
            .castTo(Trigger::new)
            .onTrue(new InstantCommand(() -> IOConstants.controller.setRumble(RumbleType.kBothRumble, Math.copySign(1.0, navxJerkY.getAsDouble()))));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.maxVelocity,
            AutoConstants.maxAcceleration
        ).setKinematics(drive.getKinematics());

        Trajectory sTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at origin, facing forwards
            new Pose2d(0, 0, new Rotation2d()),
            // Waypoints off to each side to create S shape
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing in the same direction.
            new Pose2d(3, 0, new Rotation2d()),
            config
        );

        ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.thetaControllerKp, 0, 0, AutoConstants.thetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            sTrajectory,
            drive::getPose,
            drive.getKinematics(),

            // Drive controllers
            new PIDController(AutoConstants.xControllerKp, 0, 0),
            new PIDController(AutoConstants.yControllerKp, 0, 0),
            thetaController,
            drive::setModuleStates,
            drive
        );

        // Reset odometry to the starting pose of the trajectory
        drive.resetOdometry(sTrajectory.getInitialPose());

        // Run the command, then stop.
        return swerveControllerCommand.andThen(() -> drive.drive(0, 0, 0, false, false));
    }

    private void initTelemetry(SubsystemBase[] subsystems) {
        for (SubsystemBase subsystem : subsystems) {
            SmartDashboard.putData(subsystem);
        }
    }
}
