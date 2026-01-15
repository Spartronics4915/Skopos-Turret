package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeTwoSubsystem extends SubsystemBase{
    
    SparkFlex intakeMotor = new SparkFlex(Constants.IntakeTwoConstants.INTAKE_TWO_MOTOR_ID, MotorType.kBrushless);
    SparkFlexConfig intakeConfig = new SparkFlexConfig();

    boolean intakeActive = false;

    public IntakeTwoSubsystem() {
        intakeConfig
            .inverted(Constants.IntakeTwoConstants.INTAKE_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Constants.IntakeTwoConstants.INTAKE_MOTOR_SMART_LIMIT)
            .secondaryCurrentLimit(Constants.IntakeTwoConstants.INTAKE_MOTOR_SECONDARY_LIMIT);
        
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void toggleIntake() {
        if(intakeActive == false){
            intakeActive = true;
            intakeMotor.set(Constants.IntakeTwoConstants.INTAKE_MOTOR_SET_SPEED);
        } else {
            intakeActive = false;
            intakeMotor.set(0.0);   
        }
    }
}
