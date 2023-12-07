package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is an example of a basic arm subsystem that represents and arm mechanism
 * with one motor and two limit switches at the upper and lower limits of its range of motion
 */
public class TemplateArmSubsystem extends SubsystemBase {

    /**
     * The Singleton instance of this TemplateArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static TemplateArmSubsystem INSTANCE = new TemplateArmSubsystem();

    private final WPI_TalonFX talon;
    private final DigitalInput upperLimit;
    private final DigitalInput lowerLimit;

    /**
     * Returns the Singleton instance of this TemplateArmSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code TemplateArmSubsystem.getInstance();}
     */
    public static TemplateArmSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this TemplateArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private TemplateArmSubsystem() {
        talon = new WPI_TalonFX(Constants.ARM_TALON);
        upperLimit = new DigitalInput(Constants.ARM_UPPER_LIMIT_PORT);
        lowerLimit = new DigitalInput(Constants.ARM_LOWER_LIMIT_PORT);
    }

    /**
     * Moves the arm at the specified power
     * @param power The speed at which to move the arm. Between -1 and 1.
     */
    public void moveArm(double power) {
        talon.set(power);
    }

    /**
     *
     * @return Whether the upper limit switch is being pressed.
     */
    public boolean isUpperLimitPressed() {
        return upperLimit.get();
    }


    /**
     *
     * @return Whether the lower limit switch is being pressed.
     */
    public boolean isLowerLimitPressed() {
        return lowerLimit.get();
    }
}

