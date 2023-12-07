// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindRamsete;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TemplateDrivetrainSubsystem;

import java.util.function.Consumer;
import java.util.function.Supplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    XboxController xboxController;
    PathfindRamsete ramsete;
    TemplateDrivetrainSubsystem drivetrainSubsystem;
    PathPlannerPath targetPath;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        drivetrainSubsystem = new TemplateDrivetrainSubsystem();
//       // ramsete = new PathfindRamsete(targetPath, Ideally we'd be able to set this up with real values and a real target path
//                new PathConstraints(4,4,1,1),
//                (Supplier<Pose2d>) drivetrainSubsystem::getPose, drivetrainSubsystem.getChassisSpeeds(),  drivetrainSubsystem::setChassisSpeeds, new ReplanningConfig(true, true), drivetrainSubsystem);

        configureBindings();
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        // Uses a lambda expression to set the default command of the TemplateDrivetrainSubsystem to drive
        // based on the joystick values of the Xbox controller
        TemplateDrivetrainSubsystem.getInstance().setDefaultCommand(
            new RunCommand(() -> TemplateDrivetrainSubsystem.getInstance().tankDrive(xboxController.getLeftY(), xboxController.getRightY()),
                TemplateDrivetrainSubsystem.getInstance())
        );

        // While the A button on the Xbox controller is pressed, the arm will move up at 50% power
        JoystickButton moveArmButton = new JoystickButton(xboxController, XboxController.Button.kA.value);
        moveArmButton.whileTrue(new MoveArmCommand(0.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return new InstantCommand();
    }
}
