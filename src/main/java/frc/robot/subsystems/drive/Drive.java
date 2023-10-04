package frc.robot.subsystems.drive;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.MAXSwerveModule.ModuleLabel;
import wildlib.utils.SwerveUtils;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.IOConstants;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Control subsystem used to command a swerve drive base. */
public class Drive extends SubsystemBase {
    private static Drive m_instance;
    public static Drive getInstance() {
        if (m_instance == null) {
            m_instance = new Drive();
        }

        return m_instance;
    }

    private final double base;
    private final double track;

    private final AHRS gyro;

    private final MAXSwerveModule aModule;
    private final MAXSwerveModule bModule;
    private final MAXSwerveModule cModule;
    private final MAXSwerveModule dModule;

    /** Position of the A wheel relative to the center of the drivetrain. */
    private final Translation2d aPosition;
    /** Position of the B wheel relative to the center of the drivetrain. */
    private final Translation2d bPosition;
    /** Position of the C wheel relative to the center of the drivetrain. */
    private final Translation2d cPosition;
    /** Position of the D wheel relative to the center of the drivetrain. */
    private final Translation2d dPosition;

    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.magnitudeSlewRate);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rotationalSlewRate);
    private double prevTime = WPIUtilJNI.now() * 1e-6;

    final SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;

    private final Field2d m_field = new Field2d();

    /**
     * Creates a swerve drivetrain from the given motors and dimensions.
     * Motors and dimensions are placed like so on the diagram:
     * <pre>
     * 
     * &ensp;   Track
     *  ┌─────────┐
     * ┌───────────┐
     * │A         B│  ─┐
     * │           │   │
     * │     ·     │   │ Base
     * │           │   │
     * │C         D│  ─┘
     * └───────────┘
     * 
     * </pre>
     * Track and base dimensions are measured from the centers of each wheel.
     */
    private Drive() {
        SmartDashboard.putData("Field", m_field);

        base = DriveConstants.wheelBase;
        track = DriveConstants.trackWidth;

        gyro = navxInit();

        aModule = new MAXSwerveModule(IOConstants.aPowerId, IOConstants.aRotationId, ModuleLabel.A);
        bModule = new MAXSwerveModule(IOConstants.bPowerId, IOConstants.bRotationId, ModuleLabel.B);
        cModule = new MAXSwerveModule(IOConstants.cPowerId, IOConstants.cRotationId, ModuleLabel.C);
        dModule = new MAXSwerveModule(IOConstants.dPowerId, IOConstants.dRotationId, ModuleLabel.D);

        aPosition = new Translation2d(this.base/2, this.track/2);
        bPosition = new Translation2d(this.base/2, -this.track/2);
        cPosition = new Translation2d(-this.base/2, this.track/2);
        dPosition = new Translation2d(-this.base/2, -this.track/2);
        
        kinematics = new SwerveDriveKinematics(aPosition, bPosition, cPosition, dPosition);
        odometry = new SwerveDriveOdometry(kinematics, 
            Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0)),
            new SwerveModulePosition[] {
                aModule.getPosition(),
                bModule.getPosition(),
                cModule.getPosition(),
                dModule.getPosition()   
            }
        );
    }

    /** Updates the odometry every event cycle */
    @Override
    public void periodic() {
        // Update the odometry in the periodic block.
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0)),
            new SwerveModulePosition[] {
                aModule.getPosition(),
                bModule.getPosition(),
                cModule.getPosition(),
                dModule.getPosition()
            }
        );

        m_field.setRobotPose(odometry.getPoseMeters());
    }

    
    /** 
     * Gets the recorded pose of the robot.
     * @return Current calculated pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    
    /** 
     * Resets the swerve odometry to the given pose.
     * @param pose Pose to reset the odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0)),
            new SwerveModulePosition[] {
                aModule.getPosition(),
                bModule.getPosition(),
                cModule.getPosition(),
                dModule.getPosition()
            },
            pose
        );
    }

    /**
     * Method to drive the robot using joystick info.
     * 
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (strafe).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit     Whether to enable slew rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convewrt XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(xSpeed*xSpeed + ySpeed*ySpeed);

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate = currentTranslationMag != 0.0? Math.abs(DriveConstants.directionSlewRate / currentTranslationMag) : 500.0;

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);

            if (angleDif < 0.45*Math.PI) {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85*Math.PI) {
                if (currentTranslationMag > 1e-4) {
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0.0);
            }

            prevTime = currentTime;

            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        double xSpeedDelivered = xSpeedCommanded * DriveConstants.maxTranslationalSpeed;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.maxTranslationalSpeed;
        double rotDelivered = currentRotation * DriveConstants.maxAngularSpeed;

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxTranslationalSpeed);

        aModule.setTargetState(swerveModuleStates[0]);
        bModule.setTargetState(swerveModuleStates[1]);
        cModule.setTargetState(swerveModuleStates[2]);
        dModule.setTargetState(swerveModuleStates[3]);
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void crossWheels() {
        aModule.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        bModule.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        cModule.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        dModule.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /** 
     * Sets the modules to the given SwerveModuleStates
     * 
     * @param targetStates The target SwerveModuleStates. Must be at least of length 4.
     */
    public void setModuleStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.maxTranslationalSpeed);
        aModule.setTargetState(targetStates[0]);
        bModule.setTargetState(targetStates[1]);
        cModule.setTargetState(targetStates[2]);
        dModule.setTargetState(targetStates[3]);
    }

    /** Zeros the drive encoders at their current position. */
    public void resetEncoders() {
        aModule.resetEncoders();
        bModule.resetEncoders();
        cModule.resetEncoders();
        dModule.resetEncoders();
    }

    /** Zeros the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return The robot's heading in degrees, from -180 to 180.
     */
    public double getHeading() {
        // Map continuous gyro degrees to -180 to 180
        return Rotation2d.fromDegrees(gyro.getAngle() * (DriveConstants.gyroReversed ? -1.0 : 1.0)).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return The turn rate of the robot, in degrees per second.
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.gyroReversed? -1.0: 1.0);
    }

    
    /** 
     * Gets the inner {@link edu.wpi.first.math.kinematics.SwerveDriveKinematics kinematics} object.
     * @return SwerveDriveKinematics
     */
    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
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

    public AHRS getNavx() {
        return gyro;
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory, boolean reset) {
        return new InstantCommand(() -> {
            if (reset) {
                this.resetOdometry(trajectory.getInitialHolonomicPose());
            }
        }, this)
        .andThen(new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            this.kinematics,
            new PIDController(currentTranslationDir, currentRotation, base),
            new PIDController(currentTranslationDir, currentRotation, base),
            new PIDController(currentTranslationDir, currentRotation, base),
            this::setModuleStates,
            true,
            this
        ));
    }

    public Command centerRetro(DoubleSupplier position, Subsystem... requirements) {
        PIDCommand center = new PIDCommand(new PIDController(0.05, 0.0, 0.001), position, -2.5, (double speed) -> {
            drive(0.0, speed, 0.0, false, true);
        }, this);
        center.addRequirements(requirements);
        return center;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        aModule.initSendable(builder);
        bModule.initSendable(builder);
        cModule.initSendable(builder);
        dModule.initSendable(builder);

        builder.addDoubleProperty("Drive Angle (deg)", () -> getHeading(), null);
        builder.addDoubleProperty("Drive Velocity (m/s)", () -> Math.hypot(gyro.getVelocityX(), gyro.getVelocityY()), null);
    }
}