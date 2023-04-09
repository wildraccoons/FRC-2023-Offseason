// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** 
     * Drivetrain and output control constants.
     * Most are copied from the <a href="https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java">MAXSwerve Java example</a>.
     */
    public static final class DriveConstants {
        /** 
         * Distance between centers of front and back wheels on drivetrain. 
         * Explained in more detail in {@link frc.robot.subsystems.SwerveSubsystem#SwerveSubsystem(double, double, int, int, int, int, int, int, int, int, com.kauailabs.navx.frc.AHRS) <code>new frc.robot.subsystems.SwerveSubsystem()</code>}
         */
        public static final double wheelBase = Units.inchesToMeters(26.5); // Meters (m)
        /** 
         * Distance between centers of right and left wheels on drivetrain.
         * Explained in more detail in {@link frc.robot.subsystems.SwerveSubsystem#SwerveSubsystem(double, double, int, int, int, int, int, int, int, int, com.kauailabs.navx.frc.AHRS) <code>new frc.robot.subsystems.SwerveSubsystem()</code>}
         */
        public static final double trackWidth = Units.inchesToMeters(26.5); // Meters (m)

        public static final double aAngularOffset = -Math.PI/2; // Radians (rad)
        public static final double bAngularOffset = 0; // Radians (rad)
        public static final double cAngularOffset = Math.PI; // Radians (rad)
        public static final double dAngularOffset = Math.PI/2; // Radians (rad)

        public static final double maxTranslationalSpeed = 4.8;
        public static final double maxAngularSpeed = 2 * Math.PI;

        /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
        public static final double directionSlewRate = 1.2;
        /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
        public static final double magnitudeSlewRate = 1.8;
        /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
        public static final double rotationalSlewRate = 2.0;

        public static final boolean gyroReversed = false;
    }
    
    /** 
     * Constants for each individual MAXSwerve module (wheel).
     * Includes precalculated gear ratios, PID constants, and motor info.
     * Most are copied from the <a href="https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java">MAXSwerve Java example</a>.
     */
    public static final class ModuleConstants {
        /** 
         * The MAXSwerve module can be configued with one of three pinion gears: 12T (Low), 13T (Mid), and 14T (High).
         * This changes the drive speed of the module.
         * A pinion gear with more teeth will result in a faster but weaker drivetrain.
         */
        public static final int drivePinionTeeth = 13;

        // Invert the turning envoder since the output shaft rotates in the
        // opposite direction of the spteering motor in the MAXSwerve module.
        public static final boolean encoderReversed = true;

        // MAXSwerve uses NEOs for drive motors, which are brushless.
        public static final MotorType driveMotorType = MotorType.kBrushless;
        // MAXSwerve uses NEO 550s for turn motors, which are brushless.
        public static final MotorType turnMotorType = MotorType.kBrushless;

        // Calculations required for driving motor conversion factors and feed forward.
        public static final double driveMotorFreeRps = MotorConstants.NeoFreeRpm / 60;
        public static final double wheelDiameterMeters = 0.0762;
        public static final double wheelCircumferanceMeters = wheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion.
        public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
        public static final double driveWheelFreeRps = (driveMotorFreeRps * wheelCircumferanceMeters) / driveMotorReduction;

        public static final double driveEncoderPositionFactor = (wheelDiameterMeters * Math.PI) / driveMotorReduction; // meters (m)
        public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60; // meters per second (m/s)

        public static final double turnEncoderPositionFactor = 2 * Math.PI; // radians (rad)
        public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60; // radians per second (rad/s)

        public static final double turnEncoderPositionPIDMinInput = 0; // radians (rad)
        public static final double turnEncoderPositionPIDMaxInput = turnEncoderPositionFactor; // radians (rad)

        // TODO: Tune drive motor gains
        public static final double driveKP = 0.04;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveFF = 1 / driveWheelFreeRps;
        public static final double driveMinOutput = -1;
        public static final double driveMaxOutput = 1;

        // TODO: Tune turn motor gains
        public static final double turnKP = 1;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnFF = 0;
        public static final double turnMinOutput = -1;
        public static final double turnMaxOutput = 1;
        
        public static final IdleMode driveMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode turnModorIdleMode = IdleMode.kBrake;

        public static final int driveMotorCurrentLimit = 50; // amps (A)
        public static final int turnMotorCurrentLimit = 20; //amps (A)
    }

    /** Constants relating to I/O and control such as ports, IDs, control constants, etc. */
    public static final class IOConstants {
        public static final int joystickPort = 0;
        public static final Joystick joystick = new Joystick(joystickPort);
        public static final double joystickDeadband = 0.05;

        public static final int aPowerId = 0;
        public static final int aRotationId = 1;
        public static final int bPowerId = 2;
        public static final int bRotationId = 3;
        public static final int cPowerId = 4;
        public static final int cRotationId = 5;
        public static final int dPowerId = 6;
        public static final int dRotationId = 7;
    }

    /** Various motor constants taken from datasheets. */
    public static final class MotorConstants {
        public static final double NeoFreeRpm = 5676;
        public static final double NeoEncoderResolution = 42;

        public static final double Neo550FreeRpm = 11000;
        public static final double Neo550EncoderResolution = 42;
    }

    public static final class AutoConstants {
        public static final double maxVelocity = 3; // Meters per second (m/s)
        public static final double maxAcceleration = 3; // Meters per second squared (m/s^2)

        public static final double maxAngularVelocity = Math.PI; // Radians per second (rad/s)
        public static final double maxAngularAcceleration = Math.PI; // Radians per second squared (rad/s^2)

        public static final double xControllerKp = 1;
        public static final double yControllerKp = 1;
        public static final double thetaControllerKp = 1;

        public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
            maxAngularVelocity, maxAngularAcceleration
        );
    }
}
