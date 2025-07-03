package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase;
import java.util.function.DoubleSupplier;

/**
 * Default command for manual tank drive control using Xbox controller joysticks.
 * 
 * This command provides:
 * - Independent control of left and right drive sides
 * - Automatic deadband application to prevent joystick drift
 * - Continuous operation (runs until interrupted by other commands)
 * 
 * Control Scheme:
 * - Left joystick Y-axis controls left side of robot
 * - Right joystick Y-axis controls right side of robot
 * 
 * This command will be interrupted automatically when:
 * - Any Limelight tracking button is pressed (A, B, X, Y)
 * - Any other command that requires the drivebase is scheduled
 * - Robot is disabled or enters autonomous mode
 */
public class TankDriveCommand extends Command {
    private final drivebase m_drivebase;
    private final DoubleSupplier m_leftJoystick;  // Supplier for left joystick Y value
    private final DoubleSupplier m_rightJoystick; // Supplier for right joystick Y value

    /**
     * Constructor for tank drive command.
     * 
     * @param driveSubsystem The drivebase subsystem this command will control
     * @param leftJoystick Supplier for left joystick Y-axis value
     * @param rightJoystick Supplier for right joystick Y-axis value
     */
    public TankDriveCommand(drivebase driveSubsystem, 
                          DoubleSupplier leftJoystick, 
                          DoubleSupplier rightJoystick) {
        m_drivebase = driveSubsystem;
        m_leftJoystick = leftJoystick;
        m_rightJoystick = rightJoystick;
        
        // Declare that this command requires the drivebase subsystem
        // This ensures no other commands can use the drivebase while this is running
        addRequirements(m_drivebase);
    }

    /**
     * Called repeatedly while this command is scheduled (approximately every 20ms).
     * Reads joystick values and applies them to the drivebase with deadband.
     */
    @Override
    public void execute() {
        // Get current joystick values
        double leftValue = m_leftJoystick.getAsDouble();
        double rightValue = m_rightJoystick.getAsDouble();
        
        // Apply deadband to prevent drift from imperfect joysticks
        double leftSpeed = m_drivebase.applyDeadband(leftValue);
        double rightSpeed = m_drivebase.applyDeadband(rightValue);
        
        // Send speeds to drivebase
        m_drivebase.tankDrive(leftSpeed, rightSpeed);
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
     * Returns false because this is a default command that should run continuously.
     */
    @Override
    public boolean isFinished() {
        return false; // Run continuously until interrupted
    }
}