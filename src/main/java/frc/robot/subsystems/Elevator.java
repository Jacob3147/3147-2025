package frc.robot.subsystems;


import static frc.robot.Utility.Constants.ElevatorConstants.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase
{
    public enum ElevatorState
    {
        START_1,
        START_2,
        START_3,
        NEUTRAL,
        LOADING,
        L1_CORAL,
        L2_CORAL,
        L3_CORAL,
        L4_CORAL,
        GROUND_ALGAE,
        LOW_REEF_ALGAE,
        HIGH_REEF_ALGAE,
        PROCESSOR,
        NET
    }

    public ElevatorState state = ElevatorState.START_1;
    public ElevatorState queued = ElevatorState.NEUTRAL;


    TalonFX elevator;
    TalonFXConfiguration elevator_config;
    MotionMagicExpoVoltage elevator_motion_request = new MotionMagicExpoVoltage(0);
    double elevator_position;
    double elevator_setpoint;


    TalonFX carriage;
    TalonFXConfiguration carriage_config;
    MotionMagicExpoVoltage carriage_motion_request = new MotionMagicExpoVoltage(0);
    double carriage_position;
    double carriage_setpoint;

    TalonFX coral;
    TalonFXS coral_intake;
    TalonFXConfiguration coral_config;
    TalonFXSConfiguration coral_intake_config;
    MotionMagicExpoVoltage coral_motion_request = new MotionMagicExpoVoltage(0);
    DutyCycleEncoder coral_encoder;


    SparkMax algae;
    TalonFX algae_intake_1;
    TalonFX algae_intake_2;
    SparkMaxConfig algae_config;
    TalonFXConfiguration algae_intake_1_config;
    TalonFXConfiguration algae_intake_2_config;

    ProfiledPIDController algae_PID;
    ArmFeedforward algae_FF;
    DutyCycleEncoder algae_encoder;
    double algae_position;
    double algae_setpoint;


    double coral_internal_encoder;
    double coral_position;
    double coral_setpoint;


    double algae_to_floor_distance;
    Supplier<Double> throttleSupplier;
    double throttle;

    public Elevator(Supplier<Double> throttleSupplier)
    {
        this.throttleSupplier = throttleSupplier;

        //NEO / Spark Max for algae wrist
        algae = new SparkMax(algae_ID,          MotorType.kBrushless);
        algae_config = new SparkMaxConfig();
        algae_config.inverted(false).smartCurrentLimit(20).voltageCompensation(12).idleMode(IdleMode.kBrake);
        algae.configure(algae_config,          ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Krakens and Minion / TalonFXS for everything else
        carriage       = new TalonFX(carriage_ID,        "rio");
        algae_intake_1 = new TalonFX(algae_intake_1_ID,  "rio");
        algae_intake_2 = new TalonFX(algae_intake_2_ID,  "rio");
        elevator       = new TalonFX (elevator_ID,       "rio");
        coral          = new TalonFX (coral_ID,          "rio");
        coral_intake   = new TalonFXS(coral_intake_ID,   "rio");


        carriage_config       = new TalonFXConfiguration();
        Slot0Configs carriageSlot0 = carriage_config.Slot0;
        CurrentLimitsConfigs carriageCurrentLimit = carriage_config.CurrentLimits;
        MotionMagicConfigs carriageMotionMagicConfigs =  carriage_config.MotionMagic;
        FeedbackConfigs carriageFeedback = carriage_config.Feedback;
        MotorOutputConfigs carriageOutputConfig = carriage_config.MotorOutput;
        

        elevator_config = new TalonFXConfiguration();
        Slot0Configs elevatorSlot0 = elevator_config.Slot0;
        CurrentLimitsConfigs elevatorCurrentLimit = elevator_config.CurrentLimits;
        MotionMagicConfigs elevatorMotionMagicConfigs =  elevator_config.MotionMagic;
        FeedbackConfigs elevatorFeedback = elevator_config.Feedback;
        MotorOutputConfigs elevatorOutputConfig = elevator_config.MotorOutput;


        coral_config  = new TalonFXConfiguration();
        Slot0Configs coralSlot0 = coral_config.Slot0;
        CurrentLimitsConfigs coralCurrentLimit = coral_config.CurrentLimits;
        MotionMagicConfigs coralMotionMagicConfigs =  coral_config.MotionMagic;
        FeedbackConfigs coralFeedback = coral_config.Feedback;
        MotorOutputConfigs coralOutputConfig = coral_config.MotorOutput;


        coral_intake_config  = new TalonFXSConfiguration();
        CurrentLimitsConfigs coral_intake_CurrentLimit = coral_intake_config.CurrentLimits;
        MotorOutputConfigs coral_intake_outputConfig = coral_intake_config.MotorOutput;


        algae_intake_1_config = new TalonFXConfiguration();
        CurrentLimitsConfigs algae_intake1_CurrentLimit = algae_intake_1_config.CurrentLimits;
        MotorOutputConfigs algae_intake1_outputConfig = algae_intake_1_config.MotorOutput;

        algae_intake_2_config = new TalonFXConfiguration();
        CurrentLimitsConfigs algae_intake2_CurrentLimit = algae_intake_2_config.CurrentLimits;
        MotorOutputConfigs algae_intake2_outputConfig = algae_intake_2_config.MotorOutput;

        
        carriageSlot0.withGravityType(GravityTypeValue.Arm_Cosine)
                     .withKV(carriage_KV).withKS(carriage_KS).withKG(carriage_KG)
                     .withKP(carriage_KP).withKD(carriage_KD);
        carriageCurrentLimit.withStatorCurrentLimit(10);
        carriageMotionMagicConfigs.withMotionMagicAcceleration(1)
                          .withMotionMagicCruiseVelocity(1);
        carriageFeedback.SensorToMechanismRatio = carriage_ratio;
        carriageOutputConfig.NeutralMode = NeutralModeValue.Brake;
        carriageOutputConfig.Inverted = InvertedValue.Clockwise_Positive;


        elevatorSlot0.withGravityType(GravityTypeValue.Elevator_Static)
                     .withKV(elevator_KV).withKS(elevator_KS).withKG(elevator_KG)
                     .withKP(elevator_KP).withKD(elevator_KD);
        elevatorCurrentLimit.withStatorCurrentLimit(10);
        elevatorMotionMagicConfigs.withMotionMagicAcceleration(1)
                          .withMotionMagicCruiseVelocity(1);
        elevatorFeedback.SensorToMechanismRatio = elevator_ratio;
        elevatorOutputConfig.NeutralMode = NeutralModeValue.Brake;
        elevatorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;


        coralSlot0.withGravityType(GravityTypeValue.Arm_Cosine)
                     .withKV(coral_KV).withKS(coral_KS).withKG(coral_KG)
                     .withKP(coral_KP).withKD(coral_KD);
        coralCurrentLimit.withStatorCurrentLimit(10);
        coralMotionMagicConfigs.withMotionMagicAcceleration(1)
                          .withMotionMagicCruiseVelocity(1);
        coralFeedback.SensorToMechanismRatio = coral_ratio;
        coralOutputConfig.NeutralMode = NeutralModeValue.Brake;
        coralOutputConfig.Inverted = InvertedValue.Clockwise_Positive;


        coral_intake_CurrentLimit.withStatorCurrentLimit(10);
        coral_intake_outputConfig.Inverted = InvertedValue.Clockwise_Positive;

        algae_intake1_CurrentLimit.withStatorCurrentLimit(10);
        algae_intake1_outputConfig.Inverted = InvertedValue.Clockwise_Positive;

        algae_intake2_CurrentLimit.withStatorCurrentLimit(10);
        algae_intake2_outputConfig.Inverted = InvertedValue.CounterClockwise_Positive;
        

        
        carriage.getConfigurator().apply(carriage_config);
        
        algae_intake_1.getConfigurator().apply(algae_intake_1_config);
        algae_intake_2.getConfigurator().apply(algae_intake_2_config);
        elevator      .getConfigurator().apply(elevator_config);
        coral         .getConfigurator().apply(coral_config);
        coral_intake  .getConfigurator().apply(coral_intake_config);



        carriage.setPosition(carriage_start);

        //seed coral kraken based on absolute encoder
        coral.setPosition(coral_encoder.get() + Units.degreesToRotations(coral_offset));
    }    

    @Override
    public void periodic() 
    {
        throttle = (throttleSupplier.get() + 1) / 2;

        //elevator and carriage from Kraken encoder
        elevator_position = elevator.getPosition(true).getValueAsDouble();
        carriage_position = carriage.getPosition(true).getValueAsDouble();

        //coral wrist and algae wrist from through bore
        algae_position = Units.rotationsToDegrees(algae_encoder.get()) + algae_offset;
        coral_position = Units.rotationsToDegrees(coral_encoder.get()) + coral_offset;

        //coral wrist also from Kraken encoder
        coral_internal_encoder = Units.rotationsToDegrees(coral.getPosition().getValueAsDouble());

        
        SmartDashboard.putNumber("elevator", elevator_position);
        SmartDashboard.putNumber("carriage", carriage_position);
        SmartDashboard.putNumber("algae wrist", algae_position);
        SmartDashboard.putNumber("coral wrist", coral_position);
        SmartDashboard.putNumber("coral wrist Kraken", coral_internal_encoder);
        SmartDashboard.putNumber("throttle", throttle);



        switch(state)
        {
            case START_1:
                elevator_setpoint = 0;
                carriage_setpoint = carriage_position;
                algae_setpoint = algae_position;
                coral_setpoint = coral_SP_SAFE;
                unstow();
                break;

            case START_2:
                elevator_setpoint = 0;
                carriage_setpoint = carriage_start + 0.1;
                algae_setpoint = algae_position;
                coral_setpoint = coral_SP_SAFE;
                break;

            case START_3:
                elevator_setpoint = 0;
                carriage_setpoint = carriage_start + 0.1;
                algae_setpoint = 0;
                coral_setpoint = coral_SP_SAFE;
                break;

            case NEUTRAL:
                elevator_setpoint = elevator_SP_NEUTRAL;
                carriage_setpoint = carriage_SP_NEUTRAL;
                algae_setpoint = algae_SP_NEUTRAL;
                coral_setpoint = coral_SP_SAFE;
                break;

            case LOADING:
                elevator_setpoint = elevator_SP_LOAD;
                carriage_setpoint = carriage_SP_LOAD;
                algae_setpoint = algae_SP_STOW;
                coral_setpoint = coral_SP_LOAD;
                break;

            case GROUND_ALGAE:
                elevator_setpoint = elevator_SP_ALGAE_GROUND;
                carriage_setpoint = carriage_SP_ALGAE_GROUND;
                algae_setpoint = algae_SP_ALGAE_GROUND;
                coral_setpoint = coral_SP_SAFE;
                break;

            case L1_CORAL:
                elevator_setpoint = elevator_SP_LEVEL1;
                carriage_setpoint = carriage_SP_LEVEL1;
                algae_setpoint = algae_SP_STOW;
                coral_setpoint = coral_SP_LEVEL1;
                break;

            case L2_CORAL:
                elevator_setpoint = elevator_SP_LEVEL2;
                carriage_setpoint = carriage_SP_LEVEL2;
                algae_setpoint = algae_SP_STOW;
                coral_setpoint = coral_SP_LEVEL2;
                break;

            case L3_CORAL:
                elevator_setpoint = elevator_SP_LEVEL3;
                carriage_setpoint = carriage_SP_LEVEL3;
                algae_setpoint = algae_SP_STOW;
                coral_setpoint = coral_SP_LEVEL3;
                break;

            case L4_CORAL:
                elevator_setpoint = elevator_SP_LEVEL4;
                carriage_setpoint = carriage_SP_LEVEL4;
                algae_setpoint = algae_SP_STOW;
                coral_setpoint = coral_SP_LEVEL4;
                break;

            case LOW_REEF_ALGAE:
                elevator_setpoint = elevator_SP_ALGAE_LOW;
                carriage_setpoint = carriage_SP_ALGAE_LOW;
                algae_setpoint = algae_SP_ALGAE_LOW;
                coral_setpoint = coral_SP_SAFE;
                break;

            case HIGH_REEF_ALGAE:
                elevator_setpoint = elevator_SP_ALGAE_HIGH;
                carriage_setpoint = carriage_SP_ALGAE_HIGH;
                algae_setpoint = algae_SP_ALGAE_HIGH;
                coral_setpoint = coral_SP_SAFE;
                break;

            case NET:
                elevator_setpoint = elevator_SP_NET;
                carriage_setpoint = carriage_SP_NET;
                algae_setpoint = algae_SP_NET;
                coral_setpoint = coral_SP_SAFE;
                break;

            case PROCESSOR:
                elevator_setpoint = elevator_SP_PROCESSOR;
                carriage_setpoint = carriage_SP_PROCESSOR;
                algae_setpoint = algae_SP_PROCESSOR;
                coral_setpoint = coral_SP_SAFE;
                break;

            default:
                break;
            
        }
       
    }

    public void start_coral_intake()
    {
        coral.set(0.5);
    }
    public void stop_coral_intake()
    {
        coral.set(0);
    }
    public void reverse_coral_intake()
    {
        coral.set(-0.5);
    }
    public void start_algae_intake()
    {
        algae.set(0.5);
    }
    public void stop_algae_intake()
    {
        algae.set(0);
    }
    public void reverse_algae_intake()
    {
        algae.set(-0.5);
    }
    public void execQueued()
    {
        state = queued;
    }

    public void score()
    {
        switch(state){


            case L1_CORAL:
                break;

            case L2_CORAL:
                break;

            case L3_CORAL:
                break;

            case L4_CORAL:
                break;

            case NET:
                break;

            case NEUTRAL:
                break;

            case PROCESSOR:
                break;


            case LOADING:
            case LOW_REEF_ALGAE:
            case GROUND_ALGAE:                
            case HIGH_REEF_ALGAE:
                //these states don't have a score
                break;

            case START_1:
            case START_2:
            case START_3:
                unstow();
                break;
            default:
                break;
        }
    }

    

    void unstow()
    {
        if(coral_position >= 30) 
        {
            state = ElevatorState.START_2;
        }
        if(state == ElevatorState.START_2 && carriage_position >= (carriage_start + 0.1))
        {
            state = ElevatorState.START_3;
        }
        if(state == ElevatorState.START_3 && algae_position >= 0)
        {
            state = ElevatorState.NEUTRAL;
        }
    }

    double algae_PID_out;
    double algae_FF_out;
    void algaeControl(double SP)
    {
        algae_PID_out = algae_PID.calculate(algae_position, SP);
        algae_FF_out = algae_FF.calculate(algae_PID.getSetpoint().position, algae_PID.getSetpoint().velocity);

        //algae.setVoltage(algae_PID_out + algae_FF_out);
    }

    void coralControl(double SP)
    {
        //coral.setControl(coral_motion_request);
    }

    void elevatorControl(double SP)
    {
        //elevator.setControl(elevator_motion_request);
    }

    void carriageControl(double SP)
    {
        //carriage.setControl(carriage_motion_request);
    }

}
