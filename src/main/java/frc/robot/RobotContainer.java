// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
// Swerve
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
// Constants
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// Navx-micro
import edu.wpi.first.wpilibj.I2C;

import java.util.List;

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
// Commands & DriverStation
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.drive(
                    MathUtil.applyDeadband(IOConstants.joystick.getY(), IOConstants.joystickDeadband),
                    MathUtil.applyDeadband(IOConstants.joystick.getX(), IOConstants.joystickDeadband),
                    MathUtil.applyDeadband(IOConstants.joystick.getZ(), IOConstants.joystickDeadband),
                    false, true
                ),
                drive
            ));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(IOConstants.joystick, 2)
            .whileTrue(new RunCommand( // Use whileTrue instead of onTrue to prevent the default command from running
                () -> drive.crossWheels(), 
                drive
            ));
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

    private AHRS navxInit() {
        try {
            return new AHRS(I2C.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating mavX-micro: " + ex.getMessage(), true);
            return null;
        }
    }
}
