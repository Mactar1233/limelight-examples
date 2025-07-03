package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to automatically turn the robot toward a Limelight target.
 * 
 * This command provides:
 * - Proportional control turning toward detected targets
 * - Turn-in-place motion (no forward/backward movement)
 * - Automatic stopping when no target is visible
 * - Continuous operation while button is held
 * 
 * How it works:
 * 1. Check if Limelight has a valid target
 * 2. If target found, calculate turning speed based on horizontal offset
 * 3. Apply opposite wheel speeds to turn in place
 * 4. If no target, stop all motion for safety
 * 
 * Typical usage:
 * - Bind to A button with .whileTrue()
 * - Use before scoring to ensure robot is aimed
 * - Combine with manual drive for coarse positioning
 * 
 * Safety features:
 * - Stops immediately if target is lost
 * - Releases control when button is released
 * - Cannot conflict with manual drive (proper command scheduling)
 */
public class LimelightAimCommand extends Command {
    private final drivebase m_drivebase;

    /**
     * Constructor for Limelight aim command.
     * 
     * @param driveSubsystem The drivebase subsystem this command will control
     */
    public LimelightAimCommand(drivebase driveSubsystem) {
        m_drivebase = driveSubsystem;
        
        // Declare that this command requires the drivebase subsystem
        // This will interrupt the default TankDriveCommand while running
        addRequirements(m_drivebase);
    }

    /**
     * Called repeatedly while this command is scheduled (approximately every 20ms).
     * Calculates aiming correction and applies turn-in-place motion.
     */
    @Override
    public void execute() {
        // Only proceed if Limelight can see a valid target
        if (m_drivebase.hasLimelightTarget()) {
            // Get steering correction from Limelight proportional control
            double steer = m_drivebase.getLimelightAimSpeed();
            
            // Turn in place: opposite wheel directions
            // Left wheels backward (-steer), right wheels forward (+steer)
            // This creates pure rotational motion without translation
            // Positive steer value = target is right of center = turn clockwise
            m_drivebase.tankDrive(-steer, steer);
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
