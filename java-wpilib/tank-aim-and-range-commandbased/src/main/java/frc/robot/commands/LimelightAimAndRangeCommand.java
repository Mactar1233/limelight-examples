package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;

/**
 * Command to simultaneously aim at target and drive to correct distance
 */
public class LimelightAimAndRangeCommand extends Command {
    private final drivebase m_Drivebase;

    public LimelightAimAndRangeCommand(drivebase driveSubsystem) {
        m_Drivebase = driveSubsystem;
        addRequirements(m_Drivebase);
    }

    @Override
    public void execute() {
        if (m_Drivebase.hasLimelightTarget()) {
            double steer = m_Drivebase.getLimelightAimSpeed();
            double forward = m_Drivebase.getLimelightRangeSpeed();
            // Combine steering and forward motion
            m_Drivebase.tankDrive(forward - steer, forward + steer);
        } else {
            m_Drivebase.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_Drivebase.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}