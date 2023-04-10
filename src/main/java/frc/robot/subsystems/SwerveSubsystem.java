package frc.robot.subsystems;

import frc.utils.SwerveUtils;

import static frc.robot.Constants.DriveConstants;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private final double base;
    private final double track;

    private final Gyro gyro;

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

    /**
     * Creates a swerve drivetrain from the given motors and dimensions.
     * Motors and dimensions are placed like so on the diagram:
     * <pre>
     * 
     * ‏     Track
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
     * 
     * @param trackWidth The track length indicated on the diagram.
     * @param wheelBase The width of the wheelbase indicated on the diagram.
     * @param gyro The gyro sensor used for gyroscopic positioning.
     * @param moduleA The MAXSwerve module at position A
     * @param moduleB The MAXSwerve module at position B
     * @param moduleC The MAXSwerve module at position C 
     * @param moduleD The MAXSwerve module at position D
     */
    public SwerveSubsystem(double wheelBase, double trackWidth, Gyro gyro,
        MAXSwerveModule moduleA, MAXSwerveModule moduleB,
        MAXSwerveModule moduleC, MAXSwerveModule moduleD
    ) {
        base = wheelBase;
        track = trackWidth;

        this.gyro = gyro;

        aModule = moduleA;
        bModule = moduleB;
        cModule = moduleC;
        dModule = moduleD;

        aPosition = new Translation2d(this.base/2, this.track/2);
        bPosition = new Translation2d(this.base/2, -this.track/2);
        cPosition = new Translation2d(-this.base/2, this.track/2);
        dPosition = new Translation2d(-this.base/2, -this.track/2);
        
        kinematics = new SwerveDriveKinematics(aPosition, bPosition, cPosition, dPosition);
        odometry = new SwerveDriveOdometry(kinematics, 
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                aModule.getPosition(),
                bModule.getPosition(),
                cModule.getPosition(),
                dModule.getPosition()   
            }
        );
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block.
        odometry.update(
            Rotation2d.fromDegrees(gyro.getAngle()),
            new SwerveModulePosition[] {
                aModule.getPosition(),
                bModule.getPosition(),
                cModule.getPosition(),
                dModule.getPosition()
            }
        );
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.getAngle()),
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
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxTranslationalSpeed);

        aModule.setTargetState(swerveModuleStates[0]);
        bModule.setTargetState(swerveModuleStates[1]);
        cModule.setTargetState(swerveModuleStates[2]);
        dModule.setTargetState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
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
        return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * 
     * @return The turn rate of the robot, in degrees per second.
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.gyroReversed? -1.0: 1.0);
    }

    public final SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}