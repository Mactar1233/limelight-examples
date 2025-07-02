// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.LimelightAimAndRangeCommand;
import frc.robot.commands.LimelightAimCommand;
import frc.robot.commands.LimelightRangeCommand;
import frc.robot.commands.LimelightWithManualOverrideCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.drivebase;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final drivebase m_drivebase = new drivebase();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
    // Configure default commands for subsystems
    configureDefaultCommands();
  }

 /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
      // A Button - Turn in place toward target
      m_driverController.a()
          .whileTrue(new LimelightAimCommand(m_drivebase));
  
      // B Button - Drive forward/backward toward target (range control)
      m_driverController.b()
          .whileTrue(new LimelightRangeCommand(m_drivebase));
  
      // X Button - Aim and range simultaneously
      m_driverController.x()
          .whileTrue(new LimelightAimAndRangeCommand(m_drivebase));
  
      // Y Button - Aim and range with manual override
      m_driverController.y()
          .whileTrue(new LimelightWithManualOverrideCommand(
              m_drivebase,
              () -> m_driverController.getLeftY(),
              () -> m_driverController.getRightY()
          ));
    // Additional button bindings can be added here
    // Example: Emergency stop on back button
    // new JoystickButton(m_driverController, XboxController.Button.kBack.value)
    //     .onTrue(new InstantCommand(() -> m_drivebase.stop(), m_drivebase));
  }

  /**
   * Configure default commands for subsystems
   */
  private void configureDefaultCommands() {
    // Set the default command for the drive subsystem
    // This command will run whenever no other command is using the drive subsystem
    m_drivebase.setDefaultCommand(
        new TankDriveCommand(
            m_drivebase,
            () -> m_driverController.getLeftY(),   // Left joystick Y-axis
            () -> m_driverController.getRightY()   // Right joystick Y-axis
        )
    );
  }
}

