package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
    SparkMax climbMotor;
    SparkMaxConfig climbMotorConfig;
    RelativeEncoder climbEncoder;
    double climbPosition;
    double climbSetpoint;

    ProfiledPIDController PID;
    SimpleMotorFeedforward FF;

    Supplier<Double> throttleSupplier;
    double throttle;

    double volts;

    public Climber(Supplier<Double> throttleSupplier)
    {
        climbMotor = new SparkMax(30, MotorType.kBrushless);
        climbMotorConfig = new SparkMaxConfig();

        climbMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(false).encoder.positionConversionFactor(0.01);

        climbMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbEncoder = climbMotor.getEncoder();
        

        PID = new ProfiledPIDController(0, 0, 0, new Constraints(1, 1));
        FF = new SimpleMotorFeedforward(0, 12.24);

        this.throttleSupplier = throttleSupplier;
    }

    @Override
    public void periodic() 
    {
        throttle = throttleSupplier.get();
        throttle = (Math.abs(throttle) < 0.05) ? 0 : throttle;
        climbPosition = climbEncoder.getPosition();
        SmartDashboard.putNumber("climb position", climbPosition);


        control(climbSetpoint);
    }

    public void up()
    {
        volts = 12;
    }

    public void out()
    {
        volts = -6;
    }

    public void stop()
    {
        volts = 0;
    }

    void control(double SP)
    {
        climbMotor.setVoltage(volts);


        double PIDout = PID.calculate(climbPosition, SP);
        double FFout = FF.calculate(PID.getSetpoint().velocity);
    }
}
