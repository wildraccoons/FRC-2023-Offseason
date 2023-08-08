// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Subsystems
import frc.robot.subsystems.drive.MAXSwerveModule;
import frc.robot.subsystems.drive.SwerveSubsystem;

// Constants
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
// Navx-micro
import edu.wpi.first.wpilibj.I2C;
import com.kauailabs.navx.frc.AHRS;
// Motors
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import wildlib.LimitSwitch;
import wildlib.PIDSparkMax;

import java.util.List;
import java.util.function.DoubleSupplier;

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
// Commands & DriverStation
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.utils.commands.DoubleEvent;
import frc.utils.commands.MotorCommand;
// Network Tables
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final AHRS navx = navxInit();

    private final SwerveSubsystem drive = new SwerveSubsystem(DriveConstants.wheelBase, DriveConstants.trackWidth, navx,
        new MAXSwerveModule(IOConstants.aPowerId, IOConstants.aRotationId, DriveConstants.aAngularOffset),
        new MAXSwerveModule(IOConstants.bPowerId, IOConstants.bRotationId, DriveConstants.bAngularOffset),
        new MAXSwerveModule(IOConstants.cPowerId, IOConstants.cRotationId, DriveConstants.cAngularOffset),
        new MAXSwerveModule(IOConstants.dPowerId, IOConstants.dRotationId, DriveConstants.dAngularOffset)
    );

    private final PIDSparkMax armExtension = new PIDSparkMax(IOConstants.armExtensionId, MotorType.kBrushless);
    private final PIDSparkMax armRotation = new PIDSparkMax(IOConstants.armRotationId, MotorType.kBrushless);
    private final LimitSwitch extensionLimit = new LimitSwitch(0, LimitSwitch.Polarity.NormallyClosed);
    private final PIDSparkMax grabberRotation = new PIDSparkMax(IOConstants.grabberRotationId, MotorType.kBrushless);
    private final PIDSparkMax grabberContraction = new PIDSparkMax(IOConstants.grabberContractionId, MotorType.kBrushless);
    
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

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();
        configureMotors();
        configureLEDs();

        new DoubleEvent(navxJerkX, (double jerk) -> Math.abs(jerk) > COLLISION_THRESHOLD)
            .castTo(Trigger::new)
            .onTrue(new InstantCommand(() -> IOConstants.controller.setRumble(RumbleType.kBothRumble, 1.0)));

        new DoubleEvent(navxJerkY, (double jerk) -> Math.abs(jerk) > COLLISION_THRESHOLD)
            .castTo(Trigger::new)
            .onTrue(new InstantCommand(() -> IOConstants.controller.setRumble(RumbleType.kBothRumble, Math.copySign(1.0, navxJerkY.getAsDouble()))));

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
            System.out.println("Zeroing NavX Micro");
            navx.zeroYaw();
        }));

        IOConstants.commandController.povRight()
            .onTrue(new InstantCommand(() -> grabberContraction.set(0.5)))
            .onFalse(new InstantCommand(() -> grabberContraction.stopMotor()));

        IOConstants.commandController.povLeft()
            .onTrue(new InstantCommand(() -> grabberContraction.set(-0.5)))
            .onFalse(new InstantCommand(() -> grabberContraction.stopMotor()));
            
            IOConstants.commandController.povUp()
            .onTrue(new InstantCommand(() -> grabberRotation.set(-0.2)))
            .onFalse(new InstantCommand(() -> grabberRotation.stopMotor()));

        IOConstants.commandController.povDown()
            .onTrue(new InstantCommand(() -> grabberRotation.set(0.2)))
            .onFalse(new InstantCommand(() -> grabberRotation.stopMotor()));

        new DoubleEvent(rightY, (y) -> y >= 0.05 || y <= -0.05)
            .castTo(Trigger::new)
            .whileTrue(new RunCommand(() -> armExtension.set(rightY.getAsDouble() * 0.3)))
            .onFalse(new InstantCommand(() -> armExtension.stopMotor()));

        IOConstants.commandController.y()
            .onTrue(new InstantCommand(() -> armRotation.set(-0.2)))
            .onFalse(new InstantCommand(() -> armRotation.stopMotor()));

        IOConstants.commandController.a()
            .onTrue(new InstantCommand(() -> armRotation.set(0.2)))
            .onFalse(new InstantCommand(() -> armRotation.stopMotor()));
    }

    private void configureMotors() {
        // Grabber Rotation
        grabberRotation.enableSoftLimit(SoftLimitDirection.kReverse, true);
        grabberRotation.enableSoftLimit(SoftLimitDirection.kForward, true);

        grabberRotation.setSoftLimit(SoftLimitDirection.kReverse, -2);
        grabberRotation.setSoftLimit(SoftLimitDirection.kForward, 6);

        // Grabber Contraction
        grabberContraction.enableSoftLimit(SoftLimitDirection.kForward, true);
        grabberContraction.enableSoftLimit(SoftLimitDirection.kReverse, true);

        grabberContraction.setSoftLimit(SoftLimitDirection.kReverse, -30);
        grabberContraction.setSoftLimit(SoftLimitDirection.kForward, 150);

        // Arm Rotation
        armRotation.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armRotation.setSoftLimit(SoftLimitDirection.kReverse, 0);
        armRotation.setInverted(true);
        armRotation.setIdleMode(IdleMode.kBrake);

        // Arm Extension
        armExtension.addLimitSwitch(extensionLimit, SoftLimitDirection.kReverse);
        armExtension.enableLimitSwitch(true, SoftLimitDirection.kReverse);
        armExtension.setInverted(true);

        armExtension.setIdleMode(IdleMode.kBrake);
    }

    private void configureLEDs() {
        leds = new AddressableLED(IOConstants.ledPort);

        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);
        for (var i = 0; i < LEDConstants.length; i++) {
            ledBuffer.setRGB(i, 255, 0, 255);
        }
        leds.setLength(LEDConstants.length);

        leds.setData(ledBuffer);
        leds.start();
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

    /**
     * Tries to initialize the a NavX on the MXP I2C port.
     * 
     * @return The {@code AHRS} object for the NavX.
     *         Returns {@code null} if there is an error during instantiation.
     */
    private AHRS navxInit() {
        try {
            return new AHRS(I2C.Port.kOnboard);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating mavX-micro: " + ex.getMessage(), true);
            return null;
        }
    }
}
