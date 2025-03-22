package frc.robot.commands;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Utility.Constants.DriveConstants;
import frc.robot.Utility.Constants.LocalizationConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPose extends Command
{
    CommandSwerveDrivetrain m_swerve;
    Pose2d current;
    Pose2d target;
    ChassisSpeeds alignmentSpeeds = new ChassisSpeeds();

    ArrayList<Pose2d> poses = new ArrayList<Pose2d>();

    ProfiledPIDController alignmentPID_X;
    ProfiledPIDController alignmentPID_Y;
    ProfiledPIDController alignmentPID_Theta;

    Supplier<Pose2d> poseSupplier = () -> LocalizationConstants.reef_1L;

    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    CommandJoystick joystick;

    double distance;
    int closest;

    public DriveToPose(CommandSwerveDrivetrain m_swerve, Supplier<Pose2d> poseSupplier, CommandJoystick joystick)
    {
        poses.add(LocalizationConstants.reef_1L);
        poses.add(LocalizationConstants.reef_1R);
        poses.add(LocalizationConstants.reef_2L);
        poses.add(LocalizationConstants.reef_2R);
        poses.add(LocalizationConstants.reef_3L);
        poses.add(LocalizationConstants.reef_3R);
        poses.add(LocalizationConstants.reef_4L);
        poses.add(LocalizationConstants.reef_4R);
        poses.add(LocalizationConstants.reef_5L);
        poses.add(LocalizationConstants.reef_5R);
        poses.add(LocalizationConstants.reef_6L);
        poses.add(LocalizationConstants.reef_6R);

        alignmentPID_X = new ProfiledPIDController(DriveConstants.PathKP, DriveConstants.PathKI, DriveConstants.PathKD, DriveConstants.AlignConstraints);
        alignmentPID_Y = new ProfiledPIDController(DriveConstants.PathKP, DriveConstants.PathKI, DriveConstants.PathKD, DriveConstants.AlignConstraints);
        alignmentPID_Theta = new ProfiledPIDController(DriveConstants.PathKP_Theta, 0, 0, DriveConstants.AlignConstraints_rot);
        alignmentPID_Theta.enableContinuousInput(0,2*Math.PI);

        this.m_swerve = m_swerve;
        this.poseSupplier = poseSupplier;
        this.joystick = joystick;

        request.withDriveRequestType(DriveRequestType.Velocity)
               .withSteerRequestType(SteerRequestType.Position)
               .withDesaturateWheelSpeeds(true);

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() 
    {
        Pose2d init = m_swerve.getState().Pose;
        alignmentPID_X.reset(init.getX());
        alignmentPID_Y.reset(init.getY());
        alignmentPID_Theta.reset(init.getRotation().getRadians());

        target = init.nearest(poses);

        m_swerve.field.getObject("align-target").setPose(target);
    }

    @Override
    public void execute() 
    {
        //target = poseSupplier.get();
        current = m_swerve.getState().Pose;

        alignmentSpeeds.vxMetersPerSecond = alignmentPID_X.calculate(current.getX(), target.getX());

        alignmentSpeeds.vyMetersPerSecond = alignmentPID_Y.calculate(current.getY(), target.getY());

        alignmentSpeeds.omegaRadiansPerSecond = alignmentPID_Theta.calculate(current.getRotation().getRadians(), target.getRotation().getRadians());
    
        request.withVelocityX(alignmentSpeeds.vxMetersPerSecond);
        request.withVelocityY(alignmentSpeeds.vyMetersPerSecond);
        request.withRotationalRate(alignmentSpeeds.omegaRadiansPerSecond);

        m_swerve.setControl(request);

        SmartDashboard.putNumber("x speed", alignmentSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("x error", alignmentPID_X.getPositionError());
        m_swerve.field.getObject("target").setPose(target);
        SmartDashboard.putNumber("x current", current.getX());
        SmartDashboard.putNumber("x target", target.getX());
    }

    @Override
    public boolean isFinished() 
    {

        return
        (
            alignmentPID_X.getPositionError() < 0.03 
            &&
            alignmentPID_Y.getPositionError() < 0.03 
            &&
            alignmentPID_Theta.getPositionError() < Units.degreesToRadians(1)
        );
    }

    @Override
    public void end(boolean interrupted) 
    {
        
    }
}
