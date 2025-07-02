package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.LimelightHelpers;

public class drivebase extends SubsystemBase{
    private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
    private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
    double deadband = 0.1; // Adjust deadband as necessary

    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    public drivebase() {
        m_rightMotor.setInverted(true); // Invert right motor if necessary
    }

    /**
     * Tank drive method - controls left and right sides independently
     * @param leftSpeed Speed for left side (-1.0 to 1.0)
     * @param rightSpeed Speed for right side (-1.0 to 1.0)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Arcade drive method - controls forward/backward and turning
     * @param forward Forward/backward speed (-1.0 to 1.0)
     * @param turn Turning speed (-1.0 to 1.0)
     */
    public void arcadeDrive(double forward, double turn) {
        m_drive.arcadeDrive(forward, turn);
    }

      /**
     * Stop all drive motors
     */
    public void stop() {
        m_drive.stopMotor();
    }

    /**
     * Apply deadband to joystick input to eliminate drift
     * @param value Raw joystick value
     * @return Deadbanded value
     */
    public double applyDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

     /**
     * Limelight aiming control
     * @return Turning speed to aim at target
     */
    public double getLimelightAimSpeed() {
        double kP = 0.035; // Proportional constant for aiming
        return LimelightHelpers.getTX("limelight") * kP;
    }


     /**
     * Limelight range control
     * @return Forward speed to reach target distance
     */
    public double getLimelightRangeSpeed() {
        double kP = 0.035; // Proportional constant for ranging
        return LimelightHelpers.getTY("limelight") * kP;
    }

    /**
     * Check if Limelight has a valid target
     * @return True if target is visible
     */

    public boolean hasLimelightTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Use this for any regular updates needed by the subsystem
    }

}
