package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private final WPI_TalonFX intakeMotor;
    private final DoubleSolenoid piston;

    public IntakeSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        intakeMotor = new WPI_TalonFX(Constants.INTAKE_TALON);
        piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    }

    public void rollIn(double speed){
        intakeMotor.set(speed);
    }

    public void rollOut(double speed){
        intakeMotor.set(-speed);
    }

    public void deploy(){
        piston.set(DoubleSolenoid.Value.kForward);
    }

    public void retract(){
        piston.set(DoubleSolenoid.Value.kReverse);
    }

    public void stop(){
        intakeMotor.stopMotor();
    }

}