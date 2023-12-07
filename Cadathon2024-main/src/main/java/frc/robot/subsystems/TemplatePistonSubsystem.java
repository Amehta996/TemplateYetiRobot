package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is an example subsystem that uses a piston.
 * You can take the basics of how this subsystem operates
 * and apply it to your specific use case.
 */
public class TemplatePistonSubsystem extends SubsystemBase {

    /**
     * The Singleton instance of this TemplateArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static TemplatePistonSubsystem INSTANCE = new TemplatePistonSubsystem();
    private final DoubleSolenoid piston;

    /**
     * Returns the Singleton instance of this TemplatePistonSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code TemplatePistonSubsystem.getInstance();}
     */
    public static TemplatePistonSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this TemplatePistonSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private TemplatePistonSubsystem() {
        piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FORWARD_CHANNEL, Constants.REVERSE_CHANNEL);
    }

    public void extend() {
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        piston.set(DoubleSolenoid.Value.kReverse);
    }
}

