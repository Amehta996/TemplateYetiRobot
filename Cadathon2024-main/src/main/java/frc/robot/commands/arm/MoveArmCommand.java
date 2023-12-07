package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TemplateArmSubsystem;

/**
 * This is a command that moves the arm mechanism at a specified
 * speed until it reaches its upper or lower limit.
 */
public class MoveArmCommand extends CommandBase {
    private final TemplateArmSubsystem templateArmSubsystem;
    private final double moveSpeed;

    public MoveArmCommand(double moveSpeed) {
        templateArmSubsystem = TemplateArmSubsystem.getInstance();
        this.moveSpeed = moveSpeed;
        addRequirements(this.templateArmSubsystem);
    }

    @Override
    public void execute() {
        templateArmSubsystem.moveArm(moveSpeed);
    }

    @Override
    public boolean isFinished() {
        if (moveSpeed < 0) {
            return templateArmSubsystem.isLowerLimitPressed();
        }
        if (moveSpeed > 0) {
            return templateArmSubsystem.isUpperLimitPressed();
        }

        return true;
    }

    @Override
    public void end(boolean interrupted) {
        templateArmSubsystem.moveArm(0);
    }
}
