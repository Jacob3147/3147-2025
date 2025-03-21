// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Utility.TunerConstants;
import frc.robot.Utility.Constants.LocalizationConstants;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class RobotContainer 
{

    

    private final Telemetry logger = new Telemetry(TunerConstants.kMaxSpeed);

    //private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick joystick = new CommandJoystick(0);
    private final CommandGenericHID buttonBox = new CommandGenericHID(1);
    private final CommandGenericHID buttonBox2 = new CommandGenericHID(2);

    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(TunerConstants.kMaxSpeed* 0.1)
            .withRotationalDeadband(TunerConstants.kMaxAngularRate * 0.2) // Add deadband
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            .withDesaturateWheelSpeeds(true);

    private final Elevator elevator = new Elevator(() -> joystick.getThrottle());
    private final Climber climber = new Climber(() -> joystick.getThrottle());

    private final SendableChooser<Command> autoChooser;
    Supplier<Pose2d> target_pose = () -> LocalizationConstants.reef_1L;

    private final DriveToPose drivePoseCommand = new DriveToPose(drivetrain, target_pose, joystick);
    

    public RobotContainer() {
        new EventTrigger("Go to L4").onTrue(Commands.runOnce(() -> elevator.state = ElevatorState.L4_CORAL));
        new EventTrigger("Score Coral").onTrue(Commands.run(() -> elevator.score()).withTimeout(0.5));
        new EventTrigger("Neutral").onTrue(Commands.runOnce(() -> elevator.state = ElevatorState.NEUTRAL));
        new EventTrigger("Intake").onTrue(Commands.run(() -> elevator.state = ElevatorState.LOADING).until(elevator.beam_break_supplier));
        
        
        
        configureBindings();
        while(!AutoBuilder.isConfigured())
        {
            //wait
        }
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("autoChooser", autoChooser);

        
       
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                .withVelocityX((-1 * joystick.getY()) 
                               * TunerConstants.kMaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY((-1*joystick.getX()) 
                               * TunerConstants.kMaxSpeed) // Drive left with negative X (left)
                .withRotationalRate((joystick.getTwist() > 0 ? -1 : 1)
                                    * Math.pow((-1*joystick.getTwist()),2) 
                                    * TunerConstants.kMaxAngularRate)) // Drive counterclockwise with negative X (left)
            );

            joystick.button(2).whileTrue(drivePoseCommand);

            joystick.trigger().whileTrue(Commands.run(() -> elevator.score()));

            joystick.button(7).or(joystick.button(8)).onTrue(
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric())
                .andThen(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d(3.141, 4.031, new Rotation2d(0))))));

            joystick.button(11).or(joystick.button(12)).onTrue(
                Commands.runOnce(() -> elevator.state = elevator.queued));


            joystick.button(3).whileTrue(
                Commands.startEnd(() -> elevator.manual_feed_down = true,
                                  () -> elevator.manual_feed_down = false));

            joystick.button(4).whileTrue(
                Commands.startEnd(() -> elevator.manual_feed_up = true,
                                  () -> elevator.manual_feed_up = false));
 
        /*
        buttonBox2.button(1)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_1L));
        buttonBox2.button(1)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_1R));

        buttonBox2.button(2)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_2L));
        buttonBox2.button(2)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_2R));

        buttonBox2.button(3)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_3L));
        buttonBox2.button(3)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_3R));

        buttonBox2.button(4)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_4L));
        buttonBox2.button(4)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_4R));

        buttonBox2.button(5)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_5L));
        buttonBox2.button(5)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_5R));

        buttonBox2.button(6)
            .and(buttonBox2.button(10))
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_6L));
        buttonBox2.button(6)
            .and(buttonBox2.button(10).negate())
                .onTrue(Commands.runOnce(() -> target_pose = () -> LocalizationConstants.reef_6R));
        */
        
        //Button box decides which state will be next
        buttonBox.button(1).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L4_CORAL));
        buttonBox.button(2).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L3_CORAL));
        buttonBox.button(3).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L2_CORAL));
        buttonBox.button(4).onTrue(Commands.runOnce(() -> elevator.queued = ElevatorState.L1_CORAL));
        buttonBox.button(5).onTrue(Commands.runOnce(() -> elevator.state = ElevatorState.NEUTRAL)); //5 is labeled net
        buttonBox.button(6).onTrue(Commands.runOnce(() -> elevator.state = ElevatorState.LOADING)
                                 
        .andThen(Commands.runOnce(() -> elevator.load_start_time = Timer.getFPGATimestamp())));

        buttonBox.button(7).whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.state = ElevatorState.CLIMBING),
                Commands.run(() -> climber.up())
            )); //PROCESSOR = CLIMB IN

        buttonBox.button(8).whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.state = ElevatorState.CLIMBING),
                Commands.run(() -> climber.out())
            )); //GROUND ALGAE = CLIMB OUT

        buttonBox.button(7).or(buttonBox.button(8)).onFalse(Commands.runOnce(() -> climber.stop()));

        buttonBox.button(9).whileTrue(
            Commands.startEnd(() -> elevator.state = ElevatorState.HIGH_ALGAE,
                              () -> elevator.state = ElevatorState.NEUTRAL));
        buttonBox.button(10).whileTrue(
            Commands.startEnd(() -> elevator.state = ElevatorState.LOW_ALGAE,
                              () -> elevator.state = ElevatorState.NEUTRAL));


        


        

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public void elevator_neutral()
    {
        elevator.state = ElevatorState.START_1;
    }
}
