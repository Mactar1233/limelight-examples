// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.VecBuilder;

public class Robot extends TimedRobot {
  private final NetworkTableInstance m_inst = NetworkTableInstance.getDefault();
  private final DoubleArrayTopic m_doubleArrayTopic =
      m_inst.getDoubleArrayTopic("m_doubleArrayTopic");
  
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_drive = new Drivetrain(m_doubleArrayTopic);
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  @Override
  public void autonomousPeriodic() {
    teleopPeriodic();
    m_drive.updateOdometry();
  }
  
  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }
  
  @Override
  public void robotPeriodic() {
    m_drive.periodic();
    
    // Update pose estimation with Limelight data
    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    
    if (limelightMeasurement.tagCount >= 2) {
      // High confidence measurement with multiple tags
      m_drive.addVisionMeasurement(
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
      m_drive.addVisionMeasurement(
          // Single tag measurement at close distance
          limelightMeasurement.pose, 
          limelightMeasurement.timestampSeconds,
          // This vector represents your "trust" in the pose estimate 
          // The first two numbers are in meters and represent the possible pose error, in this case 1.0m for both
          // The first number is x coordinate, the second is y. The last number is for rotation
          // This number is 9999999 as this is a large number which basically means "never trust" the rotation from the limelight pose
          VecBuilder.fill(1.0, 1.0, 9999999)
          //Single tag updates are usually more "noisy" or less accurate resulting in a larger distrust
      );
    }
    // If no tags or distant single tag, don't use vision measurement
  }
  
  @Override
  public void teleopPeriodic() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_speedLimiter.calculate(m_controller.getLeftY()) * Drivetrain.kMaxSpeed;
    
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(m_controller.getRightX()) * Drivetrain.kMaxAngularSpeed;
    
    m_drive.drive(xSpeed, rot);
  }
}