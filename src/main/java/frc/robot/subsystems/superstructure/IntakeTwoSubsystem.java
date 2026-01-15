package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class IntakeTwoSubsystem extends SubsystemBase{
    
    //SparkFlex intakeMotor = new SparkFlex(Constants.IntakeTwoConstants.INTAKE_TWO_MOTOR_ID, MotorType.kBrushless);
    //SparkFlexConfig intakeConfig = new SparkFlexConfig();
    TalonFX intakeTalon = new TalonFX(Constants.IntakeTwoConstants.INTAKE_TWO_MOTOR_ID);
    TalonFXConfigurator intakeTalonConfig = intakeTalon.getConfigurator();
    boolean intakeActive = false;

    public IntakeTwoSubsystem() {
        //SPARK FLEX CONFIG: 
        // intakeConfig.inverted(Constants.IntakeTwoConstants.INTAKE_MOTOR_INVERTED);
        // intakeConfig.idleMode(IdleMode.kBrake);
        // intakeConfig.smartCurrentLimit(Constants.IntakeTwoConstants.INTAKE_MOTOR_SMART_LIMIT);
        // intakeConfig.secondaryCurrentLimit(Constants.IntakeTwoConstants.INTAKE_MOTOR_SECONDARY_LIMIT);
        // intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //TALON CONFIG

        intakeTalonConfig.apply(new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Constants.IntakeTwoConstants.SUPPLY_LIMIT)
        .withSupplyCurrentLowerLimit(Constants.IntakeTwoConstants.LOWER_LIMIT)
        .withSupplyCurrentLowerTime(Constants.IntakeTwoConstants.TALON_LOWER_TIME)
        );

        MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        intakeTalonConfig.apply(motorOutputConfigs);


    }

    public void toggleIntake() {
        
        if (intakeActive == false){
            intakeActive = true;
            intakeTalon.set(Constants.IntakeTwoConstants.INTAKE_MOTOR_SET_SPEED);
            //intakeMotor.set(Constants.IntakeTwoConstants.INTAKE_MOTOR_SET_SPEED);
        } else {
            intakeActive = false;
            //intakeMotor.set(0.0); 
            intakeTalon.set(0.0);  
        }
    }
}
