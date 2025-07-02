package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to turn the robot toward a Limelight target
 */
public class LimelightAimCommand extends Command {
    private final drivebase m_drivebase;

    public LimelightAimCommand(drivebase driveSubsystem) {
        m_drivebase = driveSubsystem;
        addRequirements(m_drivebase);
    }

    @Override
    public void execute() {
        if (m_drivebase.hasLimelightTarget()) {
            double steer = m_drivebase.getLimelightAimSpeed();
            // Turn in place - opposite wheel directions
            m_drivebase.tankDrive(-steer, steer);
        } else {
            m_drivebase.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivebase.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until button is released
    }
}