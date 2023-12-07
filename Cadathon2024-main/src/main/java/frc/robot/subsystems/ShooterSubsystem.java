package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

    TalonFX leftMotor, rightMotor;
    BangBangController controller;
    double target = 0;
    public ShooterSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        leftMotor = new TalonFX(Constants.SHOOTER_MOTOR_1);
        rightMotor = new TalonFX(Constants.SHOOTER_MOTOR_2);
        controller = new BangBangController();
        rightMotor.follow(leftMotor);
        leftMotor.configOpenloopRamp(1);
        rightMotor.configOpenloopRamp(1);
    }

    public void setOutput(double speed) {
        leftMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stopMotor(){
        leftMotor.set(ControlMode.PercentOutput, 0);
    }

    public double getEncoder() {
        return leftMotor.getSelectedSensorPosition();
    }

    public double getRPM() {
        return leftMotor.getSelectedSensorPosition() / 2048 * Constants.SHOOTER_DIAMATER * 3.14;
    }
    public void setDesiredRPM(double target) {
        this.target = target;
    }

    public void spinToTarget(double setpoint) {
        controller.setSetpoint(setpoint);
        leftMotor.set(ControlMode.Velocity, controller.calculate(getRPM(), setpoint));
    }
}

