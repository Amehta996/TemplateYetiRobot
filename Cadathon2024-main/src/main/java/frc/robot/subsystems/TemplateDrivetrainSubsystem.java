package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Limelight;

public class TemplateDrivetrainSubsystem extends SubsystemBase {

    /**
     * The Singleton instance of this TemplateArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private static final TemplateDrivetrainSubsystem INSTANCE = new TemplateDrivetrainSubsystem();
    WPI_TalonFX leftMotor1, leftMotor2, leftMotor3;

    WPI_TalonFX rightMotor1, rightMotor2, rightMotor3;
    Limelight limelight;

    Pigeon2 gyro;
    DifferentialDrive drive;
    DifferentialDriveOdometry ddo;

    /**
     * Returns the Singleton instance of this TemplateDrivetrainSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code TemplateDrivetrainSubsystem.getInstance();}
     */
    public static TemplateDrivetrainSubsystem getInstance() {
        return INSTANCE;
    }

    public TemplateDrivetrainSubsystem() {
        leftMotor1 = new WPI_TalonFX(Constants.LEFT_DRIVE_TALON_1);
        leftMotor2 = new WPI_TalonFX(Constants.LEFT_DRIVE_TALON_2);
        leftMotor3 = new WPI_TalonFX(Constants.LEFT_DRIVE_TALON_3);
        rightMotor1 = new WPI_TalonFX(Constants.RIGHT_DRIVE_TALON_1);
        rightMotor2 = new WPI_TalonFX(Constants.RIGHT_DRIVE_TALON_2);
        rightMotor3 = new WPI_TalonFX(Constants.RIGHT_DRIVE_TALON_3);
        gyro = new Pigeon2(Constants.PIGEON);


        leftMotor1.configVoltageCompSaturation(12);
        leftMotor2.configVoltageCompSaturation(12);
        leftMotor3.configVoltageCompSaturation(12);
        rightMotor1.configVoltageCompSaturation(12);
        rightMotor2.configVoltageCompSaturation(12);
        rightMotor3.configVoltageCompSaturation(12);

        leftMotor1.enableVoltageCompensation(true);
        leftMotor2.enableVoltageCompensation(true);
        leftMotor3.enableVoltageCompensation(true);
        rightMotor1.enableVoltageCompensation(true);
        rightMotor2.enableVoltageCompensation(true);
        rightMotor3.enableVoltageCompensation(true);

        MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, leftMotor3);
        MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, rightMotor3);

        drive = new DifferentialDrive(leftMotors, rightMotors);
        ddo = new DifferentialDriveOdometry(new Rotation2d(gyro.getYaw()), leftMotor1.getSelectedSensorPosition(), rightMotor1.getSelectedSensorPosition());
    }


    @Override
    public void periodic() {
        ddo.update(new Rotation2d(gyro.getYaw()), getLeftEncoderDistance(), getRightEncoderDistance());
    }

    public void tankDrive(double leftPower, double rightPower) {
        drive.tankDrive(leftPower, rightPower);
    }

    public void autoDrive(double leftVolts, double rightVolts) {
        drive.tankDrive(leftVolts, rightVolts);
    }

    public double getLeftEncoderDistance() {
        return(leftMotor1.getSelectedSensorPosition() * 2048 / Constants.GEAR_RATIO);
    }
    public double getRightEncoderDistance() {
        return(rightMotor1.getSelectedSensorPosition() * 2048 / Constants.GEAR_RATIO);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(leftMotor1.getSelectedSensorVelocity(), 0, getYaw().getDegrees(), getYaw());
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        drive.curvatureDrive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
    }
    public double getAverageEncoders() {
        return ((getLeftEncoderDistance() + getRightEncoderDistance()) / 2);
    }

    public void resetEncoders() {
        leftMotor1.setSelectedSensorPosition(0);
        rightMotor1.setSelectedSensorPosition(0);
    }

    public Rotation2d getYaw() {
        return new Rotation2d(gyro.getYaw());
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }

    public DifferentialDriveOdometry getOdometry() {
        return ddo;
    }

    public Pose2d getPose() {
        return ddo.getPoseMeters();
    }
    public void stop() {
        drive.stopMotor();
    }


}

