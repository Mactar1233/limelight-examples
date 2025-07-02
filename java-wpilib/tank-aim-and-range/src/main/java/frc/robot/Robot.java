// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftDrive;
  private final PWMSparkMax m_rightDrive;
  private final XboxController m_Controller;
  private final DifferentialDrive m_drive;
  double leftSpeed = 0.0;
  double rightSpeed = 0.0;
  double deadband = 0.1; // Deadband threshold

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_leftDrive = new PWMSparkMax(0); // Replace with actual PWM port for left drive
    m_rightDrive = new PWMSparkMax(1); // Replace with actual PWM port for right drive
    m_Controller = new XboxController(0); // Replace with actual port for Xbox controller
    m_drive = new DifferentialDrive(m_leftDrive, m_rightDrive);

    m_rightDrive.setInverted(true);
  }

  double getLimelightAimSpeed()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    //tx is the error in angle between the center of the limelight's field of view and the center of the target
    // tx is positive when the target is to the right of the center of the limelight's field of view
    // tx is negative when the target is to the left of the center of the limelight's field of view
    //the targeting angular velocity is the speed at which the robot should turn to align with the target and will increase as the target moves farther off camera
    double targetingTurnSpeed = LimelightHelpers.getTX("limelight") * kP;

    return targetingTurnSpeed;
  }


  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double getLimelightRangeSpeed(){
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // ty ranges from (-vfov/2) to (vfov/2) in degrees. If your target is on the topmost edge of 
    // your limelight 3 feed, ty should return roughly 24 degrees.
      //tx is the error in angle between the center of the limelight's field of view and the center of the target
    // tx is positive when the target is to the right of the center of the limelight's field of view
    // tx is negative when the target is to the left of the center of the limelight's field of view
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP; //you can also use ta for this as stated in the above comment 

    return targetingForwardSpeed;
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

 /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
  double maxDriveSpeed = 1.0; // Maximum speed for the drive motors
  
  // Get the left and right joystick values from the Xbox controller
  double leftJoystickY = m_Controller.getLeftY();
  double rightJoystickY = m_Controller.getRightY();
  // Apply deadband to the joystick values to avoid drift
  if (Math.abs(leftJoystickY) < deadband) {
    leftJoystickY = 0.0;
  }
  if (Math.abs(rightJoystickY) < deadband) {
    rightJoystickY = 0.0;
  }
  leftSpeed = leftJoystickY * maxDriveSpeed;
  rightSpeed = rightJoystickY * maxDriveSpeed;
  
  // basic tank drive control
  // This sets the drive motors based on joystick input
  m_drive.tankDrive(leftSpeed, rightSpeed);
  
  // A Button - Turn in place toward target
  if(m_Controller.getAButton()){
    if(LimelightHelpers.getTV("limelight")) {
      //steer is set to the output of the getLimelightAimSpeed function
      // this will turn the robot in place toward the target
      double steer = getLimelightAimSpeed();
      m_drive.tankDrive(-steer, steer);  // CORRECTED: opposite signs for turning in place
    }
  }
  
  // B Button - Drive forward/backward toward target (range control)
  if(m_Controller.getBButton()){
    if(LimelightHelpers.getTV("limelight")) {
      // forward is set to the output of the getLimelightRangeSpeed function
      // this will drive the robot forward or backward toward the target depending on the target's distance
      double forward = getLimelightRangeSpeed();
      m_drive.tankDrive(forward, forward);  // Both wheels same direction for forward/back
    }
  }
  
  // X Button - Aim and range simultaneously
  if(m_Controller.getXButton()){
    if(LimelightHelpers.getTV("limelight")) {
      // This combines aiming and range control
      // steer is set to the output of the getLimelightAimSpeed function
      // forward is set to the output of the getLimelightRangeSpeed function
      // This allows the robot to both aim and move toward the target
      //when the robot has aimed correctly steer should be zero and when it is at the correct distance forward should be zero
      // This will drive the robot forward while turning to face the target
      double steer = getLimelightAimSpeed();
      double forward = getLimelightRangeSpeed();
      m_drive.tankDrive(forward - steer, forward + steer);  // CORRECTED: proper differential drive
    }
  }
  
  // Y Button - Aim and range with manual override
  if(m_Controller.getYButton()){
    if(LimelightHelpers.getTV("limelight")) {
      // This combines aiming and range control with manual joystick input
      // steer is set to the output of the getLimelightAimSpeed function
      // forward is set to the output of the getLimelightRangeSpeed function
      // Manual joystick input is added for fine control
      // This allows the robot to both aim and move toward the target while still allowing manual control
      //when the robot has aimed correctly steer should be zero and when it is at the correct distance forward should be zero
      // This will drive the robot forward while turning to face the target, with manual joystick input
      double steer = getLimelightAimSpeed();
      double forward = getLimelightRangeSpeed();
      
      // Add manual joystick input for fine control
      double manualLeft = leftJoystickY * 0.3;   // Reduced influence
      double manualRight = rightJoystickY * 0.3; // Reduced influence
      
      m_drive.tankDrive(forward - steer + manualLeft, forward + steer + manualRight);
    }
  }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
