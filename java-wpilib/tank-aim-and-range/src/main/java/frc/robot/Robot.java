// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a TimedRobot implementation that demonstrates Limelight vision tracking
 * with manual tank drive control. The robot can:
 * - Drive using tank drive (left/right joysticks control respective sides)
 * - Automatically aim at AprilTags or retroreflective targets using Limelight
 * - Automatically drive to optimal scoring distance using Limelight
 * - Combine aiming and ranging for complete autonomous target tracking
 * 
 * Button Controls:
 * - A Button: Turn in place to face target (aiming only)
 * - B Button: Drive forward/backward to reach target distance (ranging only)
 * - X Button: Simultaneously aim and drive to target (full auto-tracking)
 * - Y Button: Auto-tracking with manual joystick override for fine adjustments
 * 
 */
public class Robot extends TimedRobot {
  // Hardware Components
  private final PWMSparkMax m_leftDrive;   // Left side drive motor controller
  private final PWMSparkMax m_rightDrive;  // Right side drive motor controller
  private final XboxController m_Controller; // Driver controller for manual control
  private final DifferentialDrive m_drive;   // WPILib differential drive helper
  
  // Drive Control Variables
  double leftSpeed = 0.0;   // Current left side motor speed (-1.0 to 1.0)
  double rightSpeed = 0.0;  // Current right side motor speed (-1.0 to 1.0)
  double deadband = 0.1;    // Joystick deadband threshold to eliminate stick drift

  /**
   * Robot constructor - initializes all hardware components and sets up motor configurations.
   * This function is run when the robot is first started up and should be used for any
   * initialization code that needs to happen before any periodic methods are called.
   */
  public Robot() {
    // Initialize motor controllers with their respective PWM ports
    m_leftDrive = new PWMSparkMax(0);  // Left drive motor on PWM port 0
    m_rightDrive = new PWMSparkMax(1); // Right drive motor on PWM port 1
    
    // Initialize Xbox controller on USB port 0
    m_Controller = new XboxController(0);
    
    // Create differential drive object for easy tank/arcade drive control
    m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    // Invert right motor so both sides drive forward when given positive values
    // This is typically needed because motors are mounted facing opposite directions
    m_rightDrive.setInverted(true);
  }
/**
     * Calculates the rotational speed needed to aim the robot at a Limelight target.
     * Uses proportional control based on the target's horizontal offset from crosshair.
     * 
     * How Limelight Aiming Works:
     * 1. Limelight calculates 'tx' - horizontal offset of target from center of camera
     * 2. tx is positive when target is to the RIGHT of center (need to turn right)
     * 3. tx is negative when target is to the LEFT of center (need to turn left)
     * 4. Multiply tx by kP to get appropriate motor speed for correction
     * 5. Larger offset = faster turning speed for quicker correction
     * 
     * Tuning the kP value:
     * - Start with small values (0.01-0.05) for safety
     * - Too LOW: Robot turns too slowly, may never reach target
     * - Too HIGH: Robot oscillates/overshoots, never settles on target
     * - WRONG SIGN: Robot turns away from target instead of toward it
     * 
     * @return Turning speed to aim at target (-1.0 to 1.0, positive = clockwise)
     */
  double getLimelightAimSpeed() {    
    // kP (Proportional Control Constant)
    // This value determines how aggressively the robot responds to targeting errors:
    // - Too HIGH: Robot will oscillate/overshoot and never settle on target
    // - Too LOW: Robot will turn too slowly and may never reach the target
    // - WRONG SIGN: Robot will turn away from target instead of toward it
    // Start with small values (0.01-0.05) and tune up/down based on robot behavior
    double kP = .035;

    // Get horizontal offset from Limelight
    // Positive tx = target is to the right of center (robot should turn right/clockwise)
    // Negative tx = target is to the left of center (robot should turn left/counter-clockwise)
    double targetingTurnSpeed = LimelightHelpers.getTX("limelight") * kP;

    return targetingTurnSpeed;
  }

    /**
     * Calculates the forward/backward speed needed to reach optimal distance from target.
     * Uses proportional control based on the target's vertical position in camera view.
     * 
     * How Limelight Ranging Works:
     * 1. Limelight calculates 'ty' - vertical offset of target from center of camera
     * 2. ty is positive when target is ABOVE center (target is far, drive forward)
     * 3. ty is negative when target is BELOW center (target is close, drive backward)
     * 4. Multiply ty by kP to get appropriate motor speed for distance correction
     * 5. Larger offset = faster drive speed for quicker distance correction
     * 
     * Alternative Ranging Method:
     * - Use 'ta' (target area) instead of 'ty' for ranging
     * - ta increases as you get closer to target
     * - ta decreases as you get farther from target
     * - Useful when Limelight and target are at similar heights
     * - Implementation: (desiredArea - currentArea) * kP
     * 
     * Tuning the kP value:
     * - Start with small values (0.01-0.05) for safety
     * - Too LOW: Robot moves too slowly toward optimal distance
     * - Too HIGH: Robot oscillates forward/backward, never settles
     * - WRONG SIGN: Robot drives away from target instead of toward it
     * 
     * @return Forward speed to reach target distance (-1.0 to 1.0, positive = forward)
     */
  double getLimelightRangeSpeed() {
    // kP (Proportional Control Constant) for distance control
    // This value determines how aggressively the robot responds to distance errors:
    // - Too HIGH: Robot will oscillate forward/backward and never settle
    // - Too LOW: Robot will move too slowly toward optimal distance
    // - WRONG SIGN: Robot will drive away from target instead of toward it
    double kP = .035;
    //double desiredArea = 5.0; // Desired target area for optimal distance (can be tuned)
    // Uncomment the line above and use desiredArea if you want to use target area instead of ty

    // Get vertical offset from Limelight
    // Positive ty = target is above center (far away, need to drive forward)
    // Negative ty = target is below center (too close, need to drive backward)
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    
    // Alternative using target area (uncomment to use instead of ty):
    // double targetingForwardSpeed =  (desiredArea - LimelightHelpers.getTA("limelight")) * kP;

    return targetingForwardSpeed;
  }

  @Override
  public void teleopPeriodic() {
    // --- MANUAL DRIVE CONTROL ---
    double maxDriveSpeed = 1.0; // Maximum speed multiplier (0.0 to 1.0)
    
    // Get joystick inputs for tank drive
    // Left joystick Y-axis controls left side of robot
    // Right joystick Y-axis controls right side of robot
    double leftJoystickY = m_Controller.getLeftY();
    double rightJoystickY = m_Controller.getRightY();
    
    // Apply deadband to prevent robot drift from joystick imperfections
    // If joystick is barely moved (less than deadband), treat it as zero
    if (Math.abs(leftJoystickY) < deadband) {
      leftJoystickY = 0.0;
    }
    if (Math.abs(rightJoystickY) < deadband) {
      rightJoystickY = 0.0;
    }
    
    // Calculate final motor speeds
    leftSpeed = leftJoystickY * maxDriveSpeed;
    rightSpeed = rightJoystickY * maxDriveSpeed;
    
    // Apply manual tank drive control (this is the default behavior)
    m_drive.tankDrive(leftSpeed, rightSpeed);
    
    // --- LIMELIGHT AUTOMATIC CONTROLS ---
    // These button controls override manual driving when pressed
    
    // A Button: AIMING ONLY - Turn in place toward target
    if (m_Controller.getAButton()) {
      // Only proceed if Limelight can see a valid target
      if (LimelightHelpers.getTV("limelight")) {
        // Get steering correction from Limelight
        double steer = getLimelightAimSpeed();
        
        // Turn in place: left wheels backward, right wheels forward (or vice versa)
        // This makes the robot spin without moving forward/backward
        m_drive.tankDrive(-steer, steer);
      }
      // If no target is visible, robot will stop moving (no drive commands sent)
    }
    
    // B Button: RANGING ONLY - Drive forward/backward to reach target distance
    if (m_Controller.getBButton()) {
      // Only proceed if Limelight can see a valid target
      if (LimelightHelpers.getTV("limelight")) {
        // Get distance correction from Limelight
        double forward = getLimelightRangeSpeed();
        
        // Drive straight forward or backward: both wheels same direction and speed
        m_drive.tankDrive(forward, forward);
      }
      // If no target is visible, robot will stop moving
    }
    
    // X Button: FULL AUTO-TRACKING - Aim and range simultaneously
    if (m_Controller.getXButton()) {
      // Only proceed if Limelight can see a valid target
      if (LimelightHelpers.getTV("limelight")) {
        // Get both steering and distance corrections
        double steer = getLimelightAimSpeed();
        double forward = getLimelightRangeSpeed();
        
        // Combine motions using differential drive math:
        // Left side: forward motion minus steering (steering left subtracts from left side)
        // Right side: forward motion plus steering (steering right adds to right side)
        // This allows the robot to drive toward target while turning to face it
        m_drive.tankDrive(forward - steer, forward + steer);
        
        // When robot is perfectly aimed: steer = 0, only forward motion
        // When robot is at perfect distance: forward = 0, only steering motion
        // When both are correct: both steer and forward = 0, robot stops
      }
      // If no target is visible, robot will stop moving
    }
    
    // Y Button: AUTO-TRACKING WITH MANUAL OVERRIDE - Full auto plus manual fine control
    if (m_Controller.getYButton()) {
      // Only proceed if Limelight can see a valid target
      if (LimelightHelpers.getTV("limelight")) {
        // Get automatic corrections from Limelight
        double steer = getLimelightAimSpeed();
        double forward = getLimelightRangeSpeed();
        
        // Add manual joystick input for fine adjustments
        // Reduced influence (30%) so automatic control is still primary
        double manualLeft = leftJoystickY * 0.3;   // 30% influence from left joystick
        double manualRight = rightJoystickY * 0.3; // 30% influence from right joystick
        
        // Combine automatic tracking with manual input
        // This allows drivers to make fine adjustments while auto-tracking is active
        m_drive.tankDrive(forward - steer + manualLeft, forward + steer + manualRight);
        
        // The manual input is additive, so:
        // - Driver can push joysticks forward to add extra forward motion
        // - Driver can use differential joystick to add extra turning
        // - Automatic system continues to work toward target
      }
      // If no target is visible, robot will stop moving
    }
    
    // Note: If multiple buttons are pressed simultaneously, the last one processed wins
    // Consider using else-if statements if you want to prevent multiple modes at once
  }
}
  