// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  // Subsystems
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  
  // Controllers
  private final XboxController m_driverController = new XboxController(0);
  
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure default commands
    configureDefaultCommands();
  }
  
  /**
   * Configure default commands for subsystems.
   */
  private void configureDefaultCommands() {
    // Set the default command for the drive subsystem - arcade drive with slew rate limiting
    m_driveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> m_driveSubsystem.arcadeDrive(
                -m_speedLimiter.calculate(m_driverController.getLeftY()),  // Forward/backward (inverted)
                -m_rotLimiter.calculate(m_driverController.getRightX())    // Rotation (inverted)
            ),
            m_driveSubsystem
        )
    );
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Return null for no autonomous command
    return null;
  }
}