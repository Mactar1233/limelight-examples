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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * RobotContainer is the main configuration class for the robot in Command-Based programming.
 * 
 * This class is responsible for:
 * - Creating and configuring all subsystems
 * - Setting up controller input devices
 * - Binding buttons to commands (button mappings)
 * - Configuring default commands for subsystems
 * - Organizing the overall robot structure
 * 
 * In Command-Based programming, most robot logic is handled by Commands and Subsystems,
 * not in the main Robot class periodic methods. This keeps code organized and makes
 * it easier to test, debug, and maintain.
 * 
 * Robot Control Layout:
 * ┌─────────────────────────────────────────────────────────────────────────────────┐
 * │                            XBOX CONTROLLER MAPPING                              │
 * ├─────────────────────────────────────────────────────────────────────────────────┤
 * │  Default (no buttons): Tank Drive (Left Y = Left wheels, Right Y = Right wheels)│
 * │  A Button: Aim Only - Turn in place toward Limelight target                     │
 * │  B Button: Range Only - Drive forward/backward to optimal distance              │
 * │  X Button: Full Auto - Simultaneously aim and drive to target                   │
 * │  Y Button: Auto + Manual - Full auto tracking with joystick fine control        │
 * └─────────────────────────────────────────────────────────────────────────────────┘
 * 
 * Command Priority System:
 * 1. Any Limelight button command (A, B, X, Y) will interrupt manual driving
 * 2. Manual driving resumes when Limelight button is released
 * 3. Only one command can control the drivebase at a time
 * 4. Commands automatically handle starting, running, and stopping
 */
public class RobotContainer {

  //declare drivebase object 

  private final drivebase m_drivebase = new drivebase();

  //Primary driver controller - Xbox controller for all driving operations.

  private final CommandXboxController m_driverController = new CommandXboxController(0);


  /**
   * RobotContainer constructor - sets up the entire robot configuration.
   * 
   * This is called once when the robot code starts up and handles:
   * 1. Button binding configuration (which buttons do what)
   * 2. Default command configuration (what runs when no buttons are pressed)
   * 3. Any other robot-wide setup needed
   * 
   * The order of these calls matters:
   * - Button bindings should be configured before default commands
   * - Default commands should be set last to avoid conflicts
   */
  public RobotContainer() {
    // Configure button bindings first
    configureButtonBindings();
    
    // Configure default commands last
    configureDefaultCommands();
  }

  /**
   * Configures all button-to-command mappings for the robot.
   * 
   * Button Binding Types:
   * - .whileTrue(): Command runs while button is held, stops when released
   * - .onTrue(): Command starts when button is pressed, runs until finished
   * - .onFalse(): Command starts when button is released
   * - .toggleOnTrue(): Command starts/stops each time button is pressed
   * 
   * Our Configuration:
   * - All Limelight commands use .whileTrue() for immediate stop when released
   * - This provides intuitive control and immediate safety shutoff
   * - Commands automatically interrupt each other (no conflicts possible)
   * 
   * Safety Features:
   * - Commands stop immediately when buttons are released
   * - No commands can run simultaneously on the same subsystem
   * - Automatic fallback to manual control when vision is lost
   */

  private void configureButtonBindings() {
  
    /**
     * A Button: AIMING ONLY - Turn in place toward Limelight target
     * 
     * Behavior:
     * - Robot turns left/right to center target in Limelight view
     * - No forward/backward movement (turn in place)
     * - Stops immediately if target is lost or button is released
     * - Uses proportional control for smooth, accurate aiming
     * 
     * Use this when:
     * - You want to aim at target without moving closer/farther
     * - Setting up for a shot from current position
     * - Aligning robot before using other automatic functions
     * 
     * Visual indicator: Target should move toward center of Limelight view as configured in the Limelight Web Interface
     */
    m_driverController.a()
        .whileTrue(new LimelightAimCommand(m_drivebase));

    /**
     * B Button: RANGING ONLY - Drive forward/backward to optimal distance
     * 
     * Behavior:
     * - Robot drives forward/backward to reach optimal shooting distance
     * - No turning motion (straight line movement)
     * - Stops immediately if target is lost or button is released
     * - Uses proportional control based on target vertical position or area
     * 
     * Use this when:
     * - Robot is already aimed at target
     * - You want to reach optimal shooting distance
     * - Fine-tuning distance after aiming
     * 
     * Visual indicator: Target should move toward optimal vertical position as configured in the Limelight Web Interface
     */
    m_driverController.b()
        .whileTrue(new LimelightRangeCommand(m_drivebase));

    /**
     * X Button: FULL AUTOMATIC - Simultaneously aim and drive to target
     * 
     * Behavior:
     * - Robot aims at target while driving to optimal distance
     * - Combines turning and forward/backward motion intelligently
     * - Stops immediately if target is lost or button is released
     * - Most efficient way to get into perfect shooting position
     * 
     * Use this when:
     * - You want fully automatic target tracking
     * - Robot needs both aiming and distance correction
     * - Fastest way to get into shooting position
     * 
     * Visual indicator: Target should move toward center AND optimal distance as configured in the Limelight Web Interface
     */
    m_driverController.x()
        .whileTrue(new LimelightAimAndRangeCommand(m_drivebase));

    /**
     * Y Button: AUTO + MANUAL OVERRIDE - Full auto tracking with joystick fine control
     * 
     * Behavior:
     * - Robot automatically aims and ranges like X button
     * - Driver can add manual joystick input for fine adjustments (30% influence)
     * - Falls back to full manual control if target is lost
     * - Balances automatic precision with human judgment
     * 
     * Use this when:
     * - You want automatic tracking with ability to make adjustments
     * - Need precision beyond pure automatic control
     * 
     * Manual input examples:
     * - Push both sticks forward → adds extra forward motion
     * - Push left stick forward, right back → adds extra turning
     * - Release sticks → pure automatic control
     */
    m_driverController.y()
        .whileTrue(new LimelightWithManualOverrideCommand(
            m_drivebase,
            () -> m_driverController.getLeftY(),   // Left joystick supplier
            () -> m_driverController.getRightY()   // Right joystick supplier
        ));
  }
  
  /**
   * Configures default commands for all subsystems.
   * 
   * Default commands:
   * - Run automatically when no other command is using the subsystem
   * - Provide "background" functionality for subsystems
   * - Are interrupted by other commands, then resume when those commands end
   * - Should never finish on their own (return false from isFinished())
   * 
   * For the drivebase:
   * - Default command is manual tank drive using joysticks
   * - Provides normal driving when no Limelight buttons are pressed
   * - Automatically resumes when Limelight commands end
   * - Includes deadband to prevent joystick drift
   */
  private void configureDefaultCommands() {
    /**
     * Default command for drivebase: Manual tank drive control
     * 
     * This command:
     * - Runs continuously when no other commands are using the drivebase
     * - Provides normal tank drive control (left stick = left wheels, right stick = right wheels)
     * - Applies deadband to prevent drift from imperfect joysticks
     * - Is automatically interrupted by Limelight commands (A, B, X, Y buttons)
     * - Resumes automatically when Limelight commands end
     * 
     * The lambda expressions () -> m_driverController.getLeftY() create "suppliers"
     * that provide fresh joystick values each time the command runs.
     * This ensures real-time response to joystick movements.
     */
    m_drivebase.setDefaultCommand(
        new TankDriveCommand(
            m_drivebase,                              // Subsystem to control
            () -> m_driverController.getLeftY(),     // Left joystick Y-axis supplier
            () -> m_driverController.getRightY()     // Right joystick Y-axis supplier
        )
    );
  }
}