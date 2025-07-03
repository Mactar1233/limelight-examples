package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.LimelightHelpers;

/**
 * Drivebase subsystem for a tank drive robot with Limelight vision tracking capabilities.
 * 
 * This subsystem provides:
 * - Basic tank drive and arcade drive functionality
 * - Limelight-based automatic aiming (turn toward target)
 * - Limelight-based automatic ranging (drive to optimal distance)
 * - Target detection and validation
 * 
 * Hardware Configuration:
 * - Left drive motor on PWM port 0
 * - Right drive motor on PWM port 1 (inverted)
 */
public class drivebase extends SubsystemBase {
    // Hardware Components
    private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);   // Left side drive motor
    private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);  // Right side drive motor
    
    // Control Parameters
    double deadband = 0.1; // Joystick deadband threshold to eliminate stick drift
    
    // Drive Control Object
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    /**
     * Constructor for the drivebase subsystem.
     * Initializes motors and sets up proper motor inversions for tank drive.
     */
    public drivebase() {
        // Invert right motor so positive values drive both sides forward
        // This is typically needed because motors are mounted facing opposite directions
        m_rightMotor.setInverted(true);
    }

    /**
     * Tank drive method - controls left and right sides independently.
     * This is the most direct control method for differential drive robots.
     * 
     * Use this when you want:
     * - Direct control over each side of the robot
     * - Maximum precision for complex maneuvers
     * - Limelight automatic controls (combining forward and turning motions)
     * 
     * @param leftSpeed Speed for left side (-1.0 to 1.0, positive = forward)
     * @param rightSpeed Speed for right side (-1.0 to 1.0, positive = forward)
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Arcade drive method - controls forward/backward and turning with separate inputs.
     * This provides more intuitive control for human drivers.
     * 
     * Use this when you want:
     * - Intuitive single-joystick control
     * - Separate forward/backward and turning inputs
     * - Easier manual driving for operators
     * 
     * @param forward Forward/backward speed (-1.0 to 1.0, positive = forward)
     * @param turn Turning speed (-1.0 to 1.0, positive = clockwise/right)
     */
    public void arcadeDrive(double forward, double turn) {
        m_drive.arcadeDrive(forward, turn);
    }

    /**
     * Immediately stops all drive motors.
     * Use this for emergency stops or when commands end.
     */
    public void stop() {
        m_drive.stopMotor();
    }

    /**
     * Applies deadband to joystick input to eliminate drift from imperfect joysticks.
     * 
     * Joystick deadband is necessary because:
     * - Joysticks rarely return exactly to 0.0 when released
     * - Small variations can cause unwanted robot movement
     * - Deadband creates a "dead zone" around center where input is ignored
     * 
     * @param value Raw joystick value (-1.0 to 1.0)
     * @return Deadbanded value (0.0 if within deadband, otherwise original value)
     */
    public double applyDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }

    /**
     * Calculates the rotational speed needed to aim the robot at a Limelight target.
     * Uses proportional control based on the target's horizontal offset from crosshair.
     * 
     * How Limelight Aiming Works:
     * 1. Limelight calculates 'tx' - horizontal offset of target from center of camera
     * 2. tx is positive when target is to the RIGHT of center (need to turn right)
     * 3. tx is negative when target is to the LEFT of center (need to turn left)
     * 4. Multiply tx by kP to get appropriate motor speed for correction
     * 5. Larger offset = faster turning speed for quicker correction
     * 
     * Tuning the kP value:
     * - Start with small values (0.01-0.05) for safety
     * - Too LOW: Robot turns too slowly, may never reach target
     * - Too HIGH: Robot oscillates/overshoots, never settles on target
     * - WRONG SIGN: Robot turns away from target instead of toward it
     * 
     * @return Turning speed to aim at target (-1.0 to 1.0, positive = clockwise)
     */
    public double getLimelightAimSpeed() {
        // Proportional control constant for aiming
        // This determines how aggressively the robot responds to targeting errors
        double kP = 0.035;
        
        // Get horizontal offset from Limelight
        return LimelightHelpers.getTX("limelight") * kP;
    }

    /**
     * Calculates the forward/backward speed needed to reach optimal distance from target.
     * Uses proportional control based on the target's vertical position in camera view.
     * 
     * How Limelight Ranging Works:
     * 1. Limelight calculates 'ty' - vertical offset of target from center of camera
     * 2. ty is positive when target is ABOVE center (target is far, drive forward)
     * 3. ty is negative when target is BELOW center (target is close, drive backward)
     * 4. Multiply ty by kP to get appropriate motor speed for distance correction
     * 5. Larger offset = faster drive speed for quicker distance correction
     * 
     * Alternative Ranging Method:
     * - Use 'ta' (target area) instead of 'ty' for ranging
     * - ta increases as you get closer to target
     * - ta decreases as you get farther from target
     * - Useful when Limelight and target are at similar heights
     * - Implementation: (desiredArea - currentArea) * kP
     * 
     * Tuning the kP value:
     * - Start with small values (0.01-0.05) for safety
     * - Too LOW: Robot moves too slowly toward optimal distance
     * - Too HIGH: Robot oscillates forward/backward, never settles
     * - WRONG SIGN: Robot drives away from target instead of toward it
     * 
     * @return Forward speed to reach target distance (-1.0 to 1.0, positive = forward)
     */
    public double getLimelightRangeSpeed() {
        // kP (Proportional Control Constant) for distance control
        // This determines how aggressively the robot responds to distance errors
        // - Too HIGH: Robot will oscillate forward/backward and never settle
        // - Too LOW: Robot will move too slowly toward optimal distance
        // - WRONG SIGN: Robot will drive away from target instead of toward it
        double kP = 0.035;
        //double desiredArea = 5.0; // Desired target area for optimal distance (can be tuned)
            // Uncomment the line above and use desiredArea if you want to use target area instead of ty

        
        // Get vertical offset from Limelight
        // Positive ty = target is above center (far away, need to drive forward)
        // Negative ty = target is below center (too close, need to drive backward)
        return LimelightHelpers.getTY("limelight") * kP;
        
        // Alternative using target area (uncomment to use instead of ty):
        // return (desiredArea - LimelightHelpers.getTA("limelight")) * kP;
    }

    /**
     * Checks if the Limelight can see a valid target.
     * 
     * Target Validation Process:
     * 1. Limelight processes camera image using configured pipeline
     * 2. Looks for shapes/patterns matching target criteria (AprilTags, retroreflective tape, etc.)
     * 3. Returns 'tv' (target valid) as 1.0 if valid target found, 0.0 otherwise
     * 
     * Always check this before using Limelight data:
     * - Prevents erratic behavior when no target is visible
     * - Allows fallback to manual control when vision fails
     * - Ensures safety by stopping automatic movements without targets
     * 
     * @return True if Limelight has a valid target, false otherwise
     */
    public boolean hasLimelightTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    /**
     * This method is called periodically by the CommandScheduler.
     * Use this for any regular updates that need to happen regardless of which command is running.
     * 
     * Common uses:
     * - Updating sensor readings
     * - Publishing data to SmartDashboard
     * - Logging telemetry data
     */
    @Override
    public void periodic() {}
}