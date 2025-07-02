package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;
import java.util.function.DoubleSupplier;

/**
 * Tank drive command - uses left and right joysticks independently
 */
public class TankDriveCommand extends Command {
    private final drivebase m_drivebase;
    private final DoubleSupplier m_leftSpeed;
    private final DoubleSupplier m_rightSpeed;

    public TankDriveCommand(drivebase driveSubsystem, DoubleSupplier leftSpeed, DoubleSupplier rightSpeed) {
        m_drivebase = driveSubsystem;
        m_leftSpeed = leftSpeed;
        m_rightSpeed = rightSpeed;
        
        // This command requires the drive subsystem
        addRequirements(m_drivebase);
    }

    @Override
    public void execute() {
        // Apply deadband and drive with whatever value is supplied
        double left = m_drivebase.applyDeadband(m_leftSpeed.getAsDouble());
        double right = m_drivebase.applyDeadband(m_rightSpeed.getAsDouble());
        m_drivebase.tankDrive(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivebase.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }
}