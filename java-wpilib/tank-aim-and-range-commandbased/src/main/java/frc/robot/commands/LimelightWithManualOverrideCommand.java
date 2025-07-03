package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;
import java.util.function.DoubleSupplier;

/**
 * Command that combines automatic Limelight tracking with manual joystick override.
 * 
 * This command provides:
 * - Full automatic target tracking (aiming + ranging)
 * - Manual joystick input for fine adjustments
 * - Automatic fallback to manual control when no target visible
 * - Reduced manual influence to prevent fighting automatic control
 * 
 * How it works:
 * 1. Check if Limelight has a valid target
 * 2. If target found:
 *    - Calculate automatic corrections (steering + distance)
 *    - Read manual joystick inputs with reduced influence (30%)
 *    - Combine automatic and manual inputs
 *    - Apply to tank drive
 * 3. If no target found:
 *    - Fall back to full manual control
 *    - Apply deadband to joystick inputs
 * 
 * Control mixing:
 * - Automatic control: 100% influence (primary)
 * - Manual control: 30% influence (secondary)
 * - Manual input is additive to automatic control
 * - Allows fine adjustments without overwhelming automatic system
 * 
 * Use cases:
 * - Target tracking with driver assistance
 * - Fine positioning for complex scoring maneuvers
 * - Backup manual control when vision fails
 * 
 * Typical usage:
 * - Bind to Y button with .whileTrue()
 * - Use for precision target tracking with manual fine-tuning
 * 
 * Safety features:
 * - Falls back to manual control if target is lost
 * - Reduces manual influence to prevent control conflicts
 * - Releases control when button is released
 * - Cannot conflict with other drive commands (proper command scheduling)
 */
public class LimelightWithManualOverrideCommand extends Command {
    private final drivebase m_drivebase;
    private final DoubleSupplier m_leftJoystick;  // Supplier for left joystick Y value
    private final DoubleSupplier m_rightJoystick; // Supplier for right joystick Y value

    /**
     * Constructor for Limelight with manual override command.
     * 
     * @param driveSubsystem The drivebase subsystem this command will control
     * @param leftJoystick Supplier for left joystick Y-axis value
     * @param rightJoystick Supplier for right joystick Y-axis value
     */
    public LimelightWithManualOverrideCommand(drivebase driveSubsystem, 
                                            DoubleSupplier leftJoystick, 
                                            DoubleSupplier rightJoystick) {
        m_drivebase = driveSubsystem;
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;
        
        // Declare that this command requires the drivebase subsystem
        // This will interrupt the default TankDriveCommand while running
        addRequirements(m_drivebase);
    }

    /**
     * Called repeatedly while this command is scheduled (approximately every 20ms).
     * Combines automatic Limelight control with manual joystick input.
     */
    @Override
    public void execute() {
        // Check if Limelight can see a valid target
        if (m_drivebase.hasLimelightTarget()) {
            // TARGET VISIBLE: Combine automatic and manual control
            
            // Get automatic corrections from Limelight
            double steer = m_drivebase.getLimelightAimSpeed();    // Horizontal correction
            double forward = m_drivebase.getLimelightRangeSpeed(); // Distance correction
            
            // Get manual joystick inputs with deadband and reduced influence
            // 30% influence allows fine adjustments without overwhelming automatic control
            double manualLeft = m_drivebase.applyDeadband(m_leftJoystick.getAsDouble()) * 0.3;
            double manualRight = m_drivebase.applyDeadband(m_rightJoystick.getAsDouble()) * 0.3;
            
            // Combine automatic and manual inputs
            // Automatic control provides primary motion, manual provides fine adjustments
            // Left side: automatic motion + manual left adjustment
            // Right side: automatic motion + manual right adjustment
            m_drivebase.tankDrive(forward - steer + manualLeft, forward + steer + manualRight);
            
            // Example scenarios:
            // - Driver pushes both sticks forward → adds extra forward motion to automatic
            // - Driver pushes left stick forward, right stick back → adds extra turning
            // - Driver releases sticks → pure automatic control
            // - Automatic system continues working toward target regardless of manual input
        } else {
            // NO TARGET VISIBLE: Fall back to full manual control
            
            // Get joystick inputs with deadband applied
            double left = m_drivebase.applyDeadband(m_leftJoystick.getAsDouble());
            double right = m_drivebase.applyDeadband(m_rightJoystick.getAsDouble());
            
            // Apply full manual control (100% influence)
            // This provides normal tank drive when vision system can't see target
            m_drivebase.tankDrive(left, right);
        }
    }

    /**
     * Called when the command ends or is interrupted.
     * Stops the drivebase to prevent continued movement.
     */
    @Override
    public void end(boolean interrupted) {
        m_drivebase.stop();
    }

    /**
     * Determines if this command should finish automatically.
     * Returns false because this should run while button is held.
     */
    @Override
    public boolean isFinished() {
        return false; // Run until button is released
    }
}