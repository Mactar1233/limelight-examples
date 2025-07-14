// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

/**
 * Command-based drivetrain subsystem with Limelight pose estimation integration.
 */
public class DriveSubsystem extends SubsystemBase {
  // Constants
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second
  
  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;
  
  // Hardware
  private final PWMSparkMax m_leftLeader = new PWMSparkMax(1);
  private final PWMSparkMax m_leftFollower = new PWMSparkMax(2);
  private final PWMSparkMax m_rightLeader = new PWMSparkMax(3);
  private final PWMSparkMax m_rightFollower = new PWMSparkMax(4);
  
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);
  
  private final AnalogGyro m_gyro = new AnalogGyro(0);
  
  // Controllers
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  
  // Kinematics and Odometry
  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      m_leftEncoder.getDistance(),
      m_rightEncoder.getDistance(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), // State std devs
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))   // Vision std devs (default)
  );
  
  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    // Configure hardware
    configureHardware();
  }
  
  /**
   * Configure motor controllers and encoders.
   */
  private void configureHardware() {
    // Reset gyro
    m_gyro.reset();
    
    // Configure motor followers
    m_leftLeader.addFollower(m_leftFollower);
    m_rightLeader.addFollower(m_rightFollower);
    
    // Invert right side
    m_rightLeader.setInverted(true);
    
    // Configure encoders
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  
  /**
   * Drives the robot using arcade drive.
   *
   * @param xSpeed Forward speed (-1 to 1)
   * @param rot Rotation speed (-1 to 1)
   */
  public void arcadeDrive(double xSpeed, double rot) {
    // Scale inputs to max speeds
    double linearSpeed = xSpeed * kMaxSpeed;
    double angularSpeed = rot * kMaxAngularSpeed;
    
    // Convert to wheel speeds
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(linearSpeed, 0.0, angularSpeed));
    setSpeeds(wheelSpeeds);
  }
  
  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    // Calculate feedforward
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    
    // Calculate PID output
    final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
    // Set motor voltages
    m_leftLeader.setVoltage(leftOutput + leftFeedforward);
    m_rightLeader.setVoltage(rightOutput + rightFeedforward);
  }
  
  /**
   * Add a vision measurement to the pose estimator.
   *
   * @param visionRobotPose The pose of the robot as measured by the vision camera
   * @param timestampSeconds The timestamp of the vision measurement in seconds
   * @param visionMeasurementStdDevs Standard deviations of the vision measurement
   */
  public void addVisionMeasurement(Pose2d visionRobotPose, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    m_poseEstimator.addVisionMeasurement(visionRobotPose, timestampSeconds, visionMeasurementStdDevs);
  }
  
  /**
   * Updates Limelight pose estimation.
   */
  private void updateLimelightPoseEstimation() {
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    
    //Examples of filtering, this helps combat "bad" pose updates that happen 
    if (limelightMeasurement.tagCount >= 2) {
        // High confidence measurement with multiple tags
        addVisionMeasurement(
          //this is the pose returned by the limelight
          limelightMeasurement.pose, 
          //this is the timestamp the limelight got x pose
          limelightMeasurement.timestampSeconds,
          // This vector represents your "trust" in the pose estimate 
          // The first two numbers are in meters and represent the possible pose error, in this case 0.7m for both
          // The first number is x coordinate, the second is y. The last number is for rotation
          // This number is 9999999 as this is a large number which basically means "never trust" the rotation from the limelight pose
          VecBuilder.fill(0.7, 0.7, 9999999)
      );
    } else if (limelightMeasurement.tagCount == 1 && limelightMeasurement.avgTagDist < 4) {
      addVisionMeasurement(
          //this is the pose returned by the limelight
          limelightMeasurement.pose, 
          //this is the timestamp the limelight got x pose
          limelightMeasurement.timestampSeconds,
          // Single tag measurement at close distance
          // This vector represents your "trust" in the pose estimate 
          // The first two numbers are in meters and represent the possible pose error, in this case 1.0m for both
          // The first number is x coordinate, the second is y. The last number is for rotation
          // This number is 9999999 as this is a large number which basically means "never trust" the rotation from the limelight pose
          VecBuilder.fill(1.0, 1.0, 9999999)
      );
    }
    // If no tags or distant single tag, don't use vision measurement
  }
  
  @Override
  public void periodic() {
    // Update odometry
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        m_leftEncoder.getDistance(),
        m_rightEncoder.getDistance()
    );
    
    // Update Limelight pose estimation
    updateLimelightPoseEstimation();
  }
}