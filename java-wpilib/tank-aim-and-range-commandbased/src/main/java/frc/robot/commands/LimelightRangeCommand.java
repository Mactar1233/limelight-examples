package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to automatically drive forward/backward to reach optimal distance from target.
 * 
 * This command provides:
 * - Proportional control driving toward optimal shooting distance
 * - Straight-line motion (no turning)
 * - Automatic stopping when no target is visible
 * - Continuous operation while button is held
 * 
 * How it works:
 * 1. Check if Limelight has a valid target
 * 2. If target found, calculate forward speed based on vertical offset or area
 * 3. Apply same speed to both wheels for straight motion
 * 4. If no target, stop all motion for safety
 * 
 * Typical usage:
 * - Bind to B button with .whileTrue()
 * - Use after aiming to reach optimal shooting distance
 * - Combine with aiming for full automatic target tracking
 * 
 * Safety features:
 * - Stops immediately if target is lost
 * - Releases control when button is released
 * - Cannot conflict with manual drive (proper command scheduling)
 */
public class LimelightRangeCommand extends Command {
    private final drivebase m_drivebase;

    /**
     * Constructor for Limelight range command.
     * 
     * @param driveSubsystem The drivebase subsystem this command will control
     */
    public LimelightRangeCommand(drivebase driveSubsystem) {
        m_drivebase = driveSubsystem;
        
        // Declare that this command requires the drivebase subsystem
        // This will interrupt the default TankDriveCommand while running
        addRequirements(m_drivebase);
    }

    /**
     * Called repeatedly while this command is scheduled (approximately every 20ms).
     * Calculates distance correction and applies straight-line motion.
     */
    @Override
    public void execute() {
        // Only proceed if Limelight can see a valid target
        if (m_drivebase.hasLimelightTarget()) {
            // Get distance correction from Limelight proportional control
            double forward = m_drivebase.getLimelightRangeSpeed();
            
            // Drive straight forward or backward: both wheels same direction and speed
            // This creates pure translational motion without rotation
            // Positive forward value = target is far = drive forward
            // Negative forward value = target is close = drive backward
            m_drivebase.tankDrive(forward, forward);
        } else {
            // No target visible - stop for safety
            // This prevents erratic behavior when target is lost
            m_drivebase.stop();
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