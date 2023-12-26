package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;

import java.beans.DesignMode;
import java.io.PipedInputStream;
import java.util.ArrayList;
import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.FrontLeft;
import frc.robot.Constants.DriveConstants.FrontRight;
import frc.robot.Constants.DriveConstants.KeepAngle;
import frc.robot.Constants.DriveConstants.RearLeft;
import frc.robot.Constants.DriveConstants.RearRight;
import frc.robot.Constants.ModuleConstants.Drive;

/**
 * Implements a swerve Drivetrain Subsystem for the Robot
 */
public class Drivetrain extends SubsystemBase {
  private boolean fieldOriented = false;
  private double keepAngle = 0.0;
  private double timeSinceRot = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double lastDriveTime = 0.0;

  private final PIDController m_keepAnglePID = new PIDController(KeepAngle.kp, KeepAngle.ki, KeepAngle.kd);

  private final Timer m_keepAngleTimer = new Timer();

  private SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTransSlewRate);
  private SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTransSlewRate);
  private SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotSlewRate);

  private final SparkMaxSwerveModule m_FLModule = new SparkMaxSwerveModule(FrontLeft.kDrive, FrontLeft.kAziumth,
      FrontLeft.kAbsEnc, FrontLeft.kOffset);
  private final SparkMaxSwerveModule m_FRModule = new SparkMaxSwerveModule(FrontRight.kDrive, FrontRight.kAziumth,
      FrontRight.kAbsEnc, FrontRight.kOffset);
  private final SparkMaxSwerveModule m_RLModule = new SparkMaxSwerveModule(RearLeft.kDrive, RearLeft.kAziumth,
      RearLeft.kAbsEnc, RearLeft.kOffset);
  private final SparkMaxSwerveModule m_RRModule = new SparkMaxSwerveModule(RearRight.kDrive, RearRight.kAziumth,
      RearRight.kAbsEnc, RearRight.kOffset);

  private static AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics,
      ahrs.getRotation2d(), getModulePositions());

  private final SwerveDriveOdometry m_autoOdometry = new SwerveDriveOdometry(DriveConstants.kSwerveKinematics,
      ahrs.getRotation2d(), getModulePositions());

  private final double[] m_latestSlew = { 0.0, 0.0, 0.0 };

  private SwerveModuleState[] m_desStates = DriveConstants.kSwerveKinematics
      .toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));

  /**
   * Constructs a Drivetrain and resets the Gyro and Keep Angle parameters
   */
  public Drivetrain() {
    m_keepAngleTimer.reset();
    m_keepAngleTimer.start();
    m_keepAnglePID.enableContinuousInput(-Math.PI, Math.PI);
    m_odometry.resetPosition(ahrs.getRotation2d(), getModulePositions(), new Pose2d());
    ahrs.reset();

        // Configure the AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setModuleStates, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean keepAngle) {

    xSpeed = m_slewX.calculate(xSpeed);
    ySpeed = m_slewY.calculate(ySpeed);
    rot = m_slewRot.calculate(rot);

    m_latestSlew[0] = xSpeed;
    m_latestSlew[1] = ySpeed;
    m_latestSlew[2] = rot;

    if (keepAngle) {
      rot = performKeepAngle(xSpeed, ySpeed, rot); // Calls the keep angle function to update the keep angle or rotate
    }

    if (Math.abs(rot) < 0.02) {
      rot = 0.0;
    }
    if (Math.abs(xSpeed) < 0.02) {
      xSpeed  = 0.0;
    }
    if (Math.abs(ySpeed) < 0.02) {
      ySpeed = 0.0;
    }

    if (fieldRelative) {
      setModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d()));
    } else {
      setModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
    }
  }

  @Override
  public void periodic() {

    double xSpeed = getChassisSpeed().vxMetersPerSecond;
    double ySpeed = getChassisSpeed().vyMetersPerSecond;

    double speed = Math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed);

    SmartDashboard.putNumber("Speed", speed);

    SmartDashboard.putNumber("Front Left Encoder", m_FLModule.getStateAngle());
    SmartDashboard.putNumber("Front Right Encoder", m_FRModule.getStateAngle());
    SmartDashboard.putNumber("Rear Left Encoder", m_RLModule.getStateAngle());
    SmartDashboard.putNumber("Rear Right Encoder", m_RRModule.getStateAngle());

    updateOdometry();

    getPose();
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_FLModule.setDesiredState(desiredStates[0]);
    m_FRModule.setDesiredState(desiredStates[1]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  public void setModuleStates(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] desiredStates = DriveConstants.kSwerveKinematics
        .toSwerveModuleStates(secondOrderKinematics(chassisSpeeds));
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_desStates = desiredStates;
    m_FLModule.setDesiredState(desiredStates[0]);
    m_FRModule.setDesiredState(desiredStates[1]);
    m_RLModule.setDesiredState(desiredStates[2]);
    m_RRModule.setDesiredState(desiredStates[3]);
  }

  public ChassisSpeeds secondOrderKinematics(ChassisSpeeds chassisSpeeds) {
    Translation2d translation = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    Translation2d rotAdj = translation.rotateBy(new Rotation2d(-Math.PI / 2.0))
        .times(chassisSpeeds.omegaRadiansPerSecond * DriveConstants.kRotTransFactor);

    translation = translation.plus(rotAdj);

    return new ChassisSpeeds(translation.getX(), translation.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  public void stop() {
    m_FLModule.stop();
    m_FRModule.stop();
    m_RLModule.stop();
    m_RRModule.stop();
  }

  public void brakeMode(boolean enabled) {
    m_FLModule.enableBrake(enabled);
    m_FRModule.enableBrake(enabled);
    m_RLModule.enableBrake(enabled);
    m_RRModule.enableBrake(enabled);
  }

  public double getTilt() {
    return ahrs.getRoll();
    // return MathUtils.pythagorean(ahrs.getRoll(), ahrs.getPitch());
  }

  public double getTiltVel() {
    return ahrs.getRawGyroY();
  }

  /**
   * Updates odometry for the swerve drivetrain. This should be called
   * once per loop to minimize error.
   */
  public void updateOdometry() {
    m_odometry.update(ahrs.getRotation2d(), getModulePositions());
  }

  public void updateAutoOdometry() {
    m_autoOdometry.update(ahrs.getRotation2d(), getModulePositions());
  }

  /**
   * Function to retrieve latest robot gyro angle.
   * 
   * @return Rotation2d object containing Gyro angle
   */
  public Rotation2d getGyro() {
    return ahrs.getRotation2d();
  }

  /**
   * Function created to retreieve and push the robot pose to the SmartDashboard
   * for diagnostics
   * 
   * @return Pose2d object containing the X and Y position and the heading of the
   *         robot.
   */
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    Translation2d position = pose.getTranslation();

    SmartDashboard.putNumber("Robot X", position.getX());
    SmartDashboard.putNumber("Robot Y", position.getY());
    SmartDashboard.putNumber("Robot Gyro", getGyro().getRadians());

    return pose;
  }

  public Pose2d getAutoPose() {
    updateAutoOdometry();
    Pose2d pose = m_autoOdometry.getPoseMeters();
    Translation2d position = pose.getTranslation();
    return m_autoOdometry.getPoseMeters();
  }

  /**
   * Resets the odometry and gyro to the specified pose.
   *
   * @param pose in which to set the odometry and gyro.
   */
  public void resetOdometry(Pose2d pose) {
    ahrs.reset();
    ahrs.setAngleAdjustment(pose.getRotation().getDegrees());
    updateKeepAngle();
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
    m_autoOdometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  public void setPose(Pose2d pose) {
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Resets the gyro to the given angle
   * 
   * @param angle the angle of the robot to reset to
   */
  public void resetOdometry(Rotation2d angle) {
    ahrs.reset();
    ahrs.setAngleAdjustment(angle.getDegrees());
    Pose2d pose = new Pose2d(getPose().getTranslation(), angle);
    updateKeepAngle();
    m_odometry.resetPosition(ahrs.getRotation2d().times(-1.0), getModulePositions(), pose);
  }

  /**
   * Converts the 4 swerve module states into a chassisSpeed by making use of the
   * swerve drive kinematics.
   * 
   * @return ChassisSpeeds object containing robot X, Y, and Angular velocity
   */
  public ChassisSpeeds getChassisSpeed() {
    return DriveConstants.kSwerveKinematics.toChassisSpeeds(m_FLModule.getState(), m_FRModule.getState(),
        m_RLModule.getState(),
        m_RRModule.getState());
  }

  public ChassisSpeeds getCorDesChassisSpeed() {
    return DriveConstants.kSwerveKinematics.toChassisSpeeds(m_desStates[0], m_desStates[1], m_desStates[2],
        m_desStates[3]);
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] { m_FLModule.getPosition(), m_FRModule.getPosition(), m_RLModule.getPosition(),
        m_RRModule.getPosition() };
  }

  /**
   * Keep angle function is performed to combat drivetrain drift without the need
   * of constant "micro-adjustments" from the driver.
   * A PIDController is used to attempt to maintain the robot heading to the
   * keepAngle value. This value is updated when the robot
   * is rotated manually by the driver input
   * 
   * @return rotation command in radians/s
   * @param xSpeed is the input drive X speed command
   * @param ySpeed is the input drive Y speed command
   * @param rot    is the input drive rotation speed command
   */
  private double performKeepAngle(double xSpeed, double ySpeed, double rot) {
    double output = rot; // Output shouold be set to the input rot command unless the Keep Angle PID is
                         // called
    if (Math.abs(rot) >= 0.01) { // If the driver commands the robot to rotate set the
                                 // last rotate time to the current time
      lastRotTime = m_keepAngleTimer.get();
    }
    if (Math.abs(xSpeed) >= 0.01
        || Math.abs(ySpeed) >= 0.01) { // if driver commands robot to translate set the
                                       // last drive time to the current time
      lastDriveTime = m_keepAngleTimer.get();
    }
    timeSinceRot = m_keepAngleTimer.get() - lastRotTime; // update variable to the current time - the last rotate time
    timeSinceDrive = m_keepAngleTimer.get() - lastDriveTime; // update variable to the current time - the last drive
                                                             // time
    if (timeSinceRot < 0.25) { // Update keepAngle up until 0.5s after rotate command stops to allow rotation
                               // move to finish
      keepAngle = getGyro().getRadians();
    } else if (Math.abs(rot) <= 0.01 && timeSinceDrive < 0.25) { // Run Keep angle pid
                                                                 // until 0.75s after drive
                                                                 // command stops to combat
                                                                 // decel drift
      output = m_keepAnglePID.calculate(getGyro().getRadians(), keepAngle); // Set output command to the result of the
                                                                            // Keep Angle PID
    }
    return output;
  }

  public void updateKeepAngle() {
    keepAngle = getGyro().getRadians();
  }

  public void changeSlewRate(double translation, double rotation) {
    m_slewX = new SlewRateLimiter(translation, -translation, m_latestSlew[0]);
    m_slewY = new SlewRateLimiter(translation, -translation, m_latestSlew[1]);
    m_slewRot = new SlewRateLimiter(rotation, -rotation, m_latestSlew[2]);
  }
}
