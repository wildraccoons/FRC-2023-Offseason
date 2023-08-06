// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import edu.wpi.first.math.controller.PIDController;

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
         * Explained in more detail in {@link frc.robot.subsystems.drive.SwerveSubsystem#SwerveSubsystem(double, double, edu.wpi.first.wpilibj.interfaces.Gyro, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule) <code>new frc.robot.subsystems.SwerveSubsystem()</code>}
         */
        public static final double wheelBase = Units.inchesToMeters(26.5); // Meters (m)
        /** 
         * Distance between centers of right and left wheels on drivetrain.
         * Explained in more detail in {@link frc.robot.subsystems.drive.SwerveSubsystem#SwerveSubsystem(double, double, edu.wpi.first.wpilibj.interfaces.Gyro, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule, frc.robot.subsystems.MAXSwerveModule) <code>new frc.robot.subsystems.SwerveSubsystem()</code>}
         */
        public static final double trackWidth = Units.inchesToMeters(26.5); // Meters (m)

        /** Angular offset of module A (in rads). */
        public static final double aAngularOffset = -Math.PI/2;
        /** Angular offset of module B (in rads). */
        public static final double bAngularOffset = 0;
        /** Angular offset of module C (in rads). */
        public static final double cAngularOffset = Math.PI;
        /** Angular offset of module D (in rads). */
        public static final double dAngularOffset = Math.PI/2;

        /** Max translaitonal speed (in m/s) */
        public static final double maxTranslationalSpeed = 4.8;
        /** Max angular speed (in rads/s) */
        public static final double maxAngularSpeed = Math.PI;

        /** Maximum speed at which the translation vector direction ({@code Math.atan2(y, x)}) can change. Measured in rads/s. */
        public static final double directionSlewRate = 1.4; // TODO: Tune direction slew rate
        /** Maximum speed at which the translation vector magnitude ({@code Math.sqrt(x*x + y*y)}) can change. Measured in percent/s (1 = 100%). */
        public static final double magnitudeSlewRate = 1.0; // TODO: Tune magnitude slew rate
        /** Maximum speed at which the rotation vector magnitude can change. Measured in percent/s (1 = 100%). */
        public static final double rotationalSlewRate = 1.0; // TODO: Tune rotational slew rate

        /** Used to invert the gyroscope direction. */
        public static final boolean gyroReversed = true;
        /** Max input speed. */
        public static final double speed = 0.5;
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

        /**
         *  Invert the turning envoder since the output shaft rotates in the
         *  opposite direction of the steering motor in the MAXSwerve module. 
         */
        public static final boolean encoderReversed = true;

        /** MAXSwerve uses NEOs for drive motors, which are brushless. */
        public static final MotorType driveMotorType = MotorType.kBrushless;
        /** MAXSwerve uses NEO 550s for turn motors, which are brushless. */
        public static final MotorType turnMotorType = MotorType.kBrushless;

        // Calculations required for driving motor conversion factors and feed forward.
        /** Drive motor free speed (in RPS). */
        public static final double driveMotorFreeRps = MotorConstants.NeoFreeRpm / 60;
        /** Wheel diameter in meters. */
        public static final double wheelDiameterMeters = 0.0762;
        /** Wheel circumferance in meters. */
        public static final double wheelCircumferanceMeters = wheelDiameterMeters * Math.PI;
        
        /** 
         * Gear reduction on the drive motor.
         * 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion. 
         */
        public static final double driveMotorReduction = (45.0 * 22) / (drivePinionTeeth * 15);
        /** Drive wheel free speed (in RPS). */
        public static final double driveWheelFreeRps = (driveMotorFreeRps * wheelCircumferanceMeters) / driveMotorReduction;
        /** Distance traveled per rotation of the drive motor (in meters). */
        public static final double driveEncoderPositionFactor = wheelCircumferanceMeters / driveMotorReduction;
        /** Velocity factor for the drive motor (in m/s). */
        public static final double driveEncoderVelocityFactor = driveEncoderPositionFactor / 60;

        /** Radians travled per rotation of the turn encoder (absolute). In radians. */
        public static final double turnEncoderPositionFactor = 2 * Math.PI;
        /** Angular velocity factor for the turn motor (in rads/s). */
        public static final double turnEncoderVelocityFactor = turnEncoderPositionFactor / 60;

        /** Minimum value of the absolute turn encoder. */
        public static final double turnEncoderPositionPIDMinInput = 0;
        /** Maximum value of the absolute turn encoder. */
        public static final double turnEncoderPositionPIDMaxInput = turnEncoderPositionFactor; // radians (rad)

        public static final double driveKP = 0.05;
        public static final double driveKI = 0;
        public static final double driveKD = 0;
        public static final double driveFF = 1 / driveWheelFreeRps;
        public static final double driveMinOutput = -1;
        public static final double driveMaxOutput = 1;

        public static final double turnKP = 1;
        public static final double turnKI = 0;
        public static final double turnKD = 0;
        public static final double turnFF = 0;
        public static final double turnMinOutput = -1;
        public static final double turnMaxOutput = 1;
        
        /** Drive motor idle mode for Spark Max. */
        public static final IdleMode driveMotorIdleMode = IdleMode.kBrake;
        /** Rotation motor idle mode for Spark Max. */
        public static final IdleMode turnMotorIdleMode = IdleMode.kBrake;

        /** Drive motor smart current limit (in Amps). */
        public static final int driveMotorCurrentLimit = 50; // amps (A)
        /** Rotation motor smart current limit (in Amps).  */
        public static final int turnMotorCurrentLimit = 20; //amps (A)
    }

    /** Constants relating to I/O and control such as ports, IDs, control constants, etc. */
    public static final class IOConstants {
        /** XBOX controller port. */
        public static final int controllerPort = 0;
        /** XBOX controller */
        public static final XboxController controller = new XboxController(controllerPort);
        /** Command based XBOX controller. */
        public static final CommandXboxController commandController = new CommandXboxController(controllerPort);
        /** Used to invert the X and Y inputs of the controller. */
        public static final boolean xyInverted = true;
        /** Used to invert the rotation input of the controller. */
        public static final boolean rotInverted = true;

        /** Minimum movement of the joystick before moving in the X or Y directions. */
        public static final double translationDeadband = 0.05;
        /** Minimum movement of the joystick before turning. */
        public static final double rotationDeadband = 0.05;

        // CAN IDs
        /** Spark Max CAN ID for the power motor on module A */
        public static final int aPowerId = 1;
        /** Spark Max CAN ID for the direction motor on module A */
        public static final int aRotationId = 2;
        /** Spark Max CAN ID for the power motor on module B */
        public static final int bPowerId = 3;
        /** Spark Max CAN ID for the direction motor on module B */
        public static final int bRotationId = 4;
        /** Spark Max CAN ID for the power motor on module C */
        public static final int cPowerId = 5;
        /** Spark Max CAN ID for the direction motor on module C */
        public static final int cRotationId = 6;
        /** Spark Max CAN ID for the power motor on module D */
        public static final int dPowerId = 7;
        /** Spark Max CAN ID for the direction motor on module D */
        public static final int dRotationId = 8;

        /** Spark Max CAN ID for the arm rotation motor. */
        public static final int armRotationId = 11;
        /** Spark Max CAN ID for the arm extension motor. */
        public static final int armExtensionId = 12;
        /** Spark Max CAN ID for the grabber rotation motor. */
        public static final int grabberRotationId = 13;
        /** Spark Max CAN ID for the grabber contraction. */
        public static final int grabberContractionId = 14;
        /** PWM port for the addressable LEDs. */
        public static final int ledPort = 9;
    }

    /** Various motor constants taken from datasheets. */
    public static final class MotorConstants {
        /** Free speed of a NEO motor (in RPM). */
        public static final double NeoFreeRpm = 5676;
        /** Resolution of the relative encoder in a NEO motor. */
        public static final double NeoEncoderResolution = 42;
        /** Free speed of a NEO 550 motor (in RPM). */
        public static final double Neo550FreeRpm = 11_000;
        /** Resolution of the relative encoder in a NEO 550 motor. */
        public static final double Neo550EncoderResolution = 42;
    }

    /** Contants used for auto mode. */
    public static final class AutoConstants {
        /** Max velocity in auto (in m/s). */
        public static final double maxVelocity = 3;
        /** Max acceleration in auto (in m/s²) */
        public static final double maxAcceleration = 3;
        /** Max angular velociy in auto (in rad/s). */
        public static final double maxAngularVelocity = Math.PI;
        /** Max angular acceleration in auto (in rad/s²) */
        public static final double maxAngularAcceleration = Math.PI;

        /** Auto PID controller kP for Y translation.  */
        public static final double xControllerKp = 1;
        /** Auto PID controller kP for Y translation. */
        public static final double yControllerKp = 1;
        /** Auto PID controller kP for rotation. */
        public static final double thetaControllerKp = 1;

        /** Trapezoidal profile for auto rotation. */
        public static final TrapezoidProfile.Constraints thetaControllerConstraints = new TrapezoidProfile.Constraints(
            maxAngularVelocity, maxAngularAcceleration
        );
    }

    /** Contstants used for vision pipelines. */
    public static final class VisionConstants {
        /** Retroreflector PID controller kP. */
        public static final double turnKP = 0.35;
        /** Retroreflector PID controller kD. */
        public static final double turnKD = 0.01;
        /** Retroreflector PID controller kI. */
        public static final double turnKI = 0.0;

        /** Retroreflector rotation PID controller. */
        public static final PIDController turnPID = new PIDController(turnKP, turnKI, turnKD);

        /** Limelight right FOV. */
        public static final double rightFOV = 32.530;
        /** Limelight left FOV. */
        public static final double leftFOV = 30.015;
    }

    /** Constants used for addressable LEDs */
    public static final class LEDConstants {
        public static final int length = 300;
    }
}
