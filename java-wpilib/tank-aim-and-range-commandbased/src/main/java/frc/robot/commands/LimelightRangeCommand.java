package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to drive forward/backward to reach target distance
 */
public class LimelightRangeCommand extends Command {
    private final drivebase m_drivebase;

    public LimelightRangeCommand(drivebase driveSubsystem) {
        m_drivebase = driveSubsystem;
        addRequirements(m_drivebase);
    }

    @Override
    public void execute() {
        if (m_drivebase.hasLimelightTarget()) {
            double forward = m_drivebase.getLimelightRangeSpeed();
            // Drive forward/backward - both wheels same direction
            m_drivebase.tankDrive(forward, forward);
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
        return false;
    }
}