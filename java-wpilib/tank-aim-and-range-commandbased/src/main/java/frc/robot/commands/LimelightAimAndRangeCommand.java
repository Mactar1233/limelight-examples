package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to simultaneously aim at target and drive to correct distance.
 * 
 * This command provides:
 * - Combined aiming and distance control in one operation
 * - Proportional control for both steering and forward motion
 * - Automatic stopping when no target is visible
 * - Continuous operation while button is held
 * 
 * How it works:
 * 1. Check if Limelight has a valid target
 * 2. Calculate both steering correction (horizontal offset) and distance correction (vertical offset)
 * 3. Combine corrections using differential drive math
 * 4. Apply to tank drive for simultaneous aiming and ranging
 * 
 * Differential drive math:
 * - Left wheel: forward - steer (steering left reduces left wheel speed)
 * - Right wheel: forward + steer (steering right increases right wheel speed)
 * - This allows robot to drive toward target while turning to face it
 * 
 * Convergence behavior:
 * - When perfectly aimed: steer = 0, only forward motion remains
 * - When at perfect distance: forward = 0, only steering motion remains
 * - When both perfect: steer = 0 and forward = 0, robot stops automatically
 * 
 * Typical usage:
 * - Bind to X button with .whileTrue()
 * - Use for fully automatic target tracking
 * - Most efficient way to get into shooting position
 * 
 * Safety features:
 * - Stops immediately if target is lost
 * - Releases control when button is released
 * - Cannot conflict with manual drive (proper command scheduling)
 */
public class LimelightAimAndRangeCommand extends Command {
    private final drivebase m_drivebase;

    /**
     * Constructor for combined aim and range command.
     * 
     * @param driveSubsystem The drivebase subsystem this command will control
     */
    public LimelightAimAndRangeCommand(drivebase driveSubsystem) {
        m_drivebase = driveSubsystem;
        
        // Declare that this command requires the drivebase subsystem
        // This will interrupt the default TankDriveCommand while running
        addRequirements(m_drivebase);
    }

    /**
     * Called repeatedly while this command is scheduled (approximately every 20ms).
     * Calculates both aiming and distance corrections and combines them.
     */
    @Override
    public void execute() {
        // Only proceed if Limelight can see a valid target
        if (m_drivebase.hasLimelightTarget()) {
            // Get corrections from Limelight proportional controllers
            double steer = m_drivebase.getLimelightAimSpeed();    // Horizontal correction
            double forward = m_drivebase.getLimelightRangeSpeed(); // Distance correction
            
            // Combine steering and forward motion using differential drive
            // This math allows simultaneous forward motion and turning
            // Left side: forward motion minus steering (steering left subtracts)
            // Right side: forward motion plus steering (steering right adds)
            m_drivebase.tankDrive(forward - steer, forward + steer);
            
            // Example scenarios:
            // Target far and right: forward > 0, steer > 0 → left wheel slower, right wheel faster
            // Target close and left: forward < 0, steer < 0 → both wheels backward, left faster
            // Target perfectly aimed: steer = 0 → both wheels same speed (straight motion)
            // Target at perfect distance: forward = 0 → opposite wheel speeds (turn in place)
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
