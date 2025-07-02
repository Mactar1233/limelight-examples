package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;
import java.util.function.DoubleSupplier;

public class LimelightWithManualOverrideCommand extends Command {
    private final drivebase m_drivebase;
    private final DoubleSupplier m_leftJoystick;
    private final DoubleSupplier m_rightJoystick;

    public LimelightWithManualOverrideCommand(drivebase driveSubsystem, 
                                            DoubleSupplier leftJoystick, 
                                            DoubleSupplier rightJoystick) {
        m_drivebase = driveSubsystem;
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;
        addRequirements(m_drivebase);
    }

    @Override
    public void execute() {
        if (m_drivebase.hasLimelightTarget()) {
            double steer = m_drivebase.getLimelightAimSpeed();
            double forward = m_drivebase.getLimelightRangeSpeed();
            
            // Add manual control with reduced influence
            double manualLeft = m_drivebase.applyDeadband(m_leftJoystick.getAsDouble()) * 0.3;
            double manualRight = m_drivebase.applyDeadband(m_rightJoystick.getAsDouble()) * 0.3;
            
            m_drivebase.tankDrive(forward - steer + manualLeft, forward + steer + manualRight);
        } else {
            // No target - fall back to manual control
            double left = m_drivebase.applyDeadband(m_leftJoystick.getAsDouble());
            double right = m_drivebase.applyDeadband(m_rightJoystick.getAsDouble());
            m_drivebase.tankDrive(left, right);
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