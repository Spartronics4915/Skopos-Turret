package frc.robot.subsystems.superstructure;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.IntakeConstants.*;

import frc.robot.Constants.IntakeConstants.IntakeSpeed;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

public class IntakeSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private SparkMax motor;
    private SparkClosedLoopController closedLoopController;
    @SuppressWarnings("unused")
    private RelativeEncoder encoder; 
    public double setpoint = 0.0;
    public DoublePublisher intakeVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake_Current_Velocity").publish();
    public DoublePublisher intakeDesiredVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Intake_Desired_Velocity").publish();

    public IntakeSubsystem() {
        motor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        closedLoopController = motor.getClosedLoopController();
        encoder = motor.getEncoder();
        SmartDashboard.putData("ZIPLE ZAP!", setSetpointCommand(10));
        SmartDashboard.putData("ZIPLE!", setSetpointCommand(0.2));
        SmartDashboard.putData("ZUNPLE ZINK!", setSetpointCommand(0));
        SmartDashboard.putData("Bogos Binted", setSetpointCommand(-0.5));
    }

    @Override
    public void periodic() {
        setSpeed(setpoint);
        intakeVelocityPublisher.accept(getSpeed());
        intakeDesiredVelocityPublisher.accept(setpoint);
    }

    public void setSpeed(double newSpeed) {
        closedLoopController.setSetpoint(
            newSpeed,
            ControlType.kVelocity
        );
        setpoint = newSpeed;
    }

    public double getSpeed(){
        return motor.getEncoder().getVelocity();
    }

    public void intakeMotors(IntakeSpeed preset) {
        setSpeed(preset.intakeSpeed);
    }

    public Command setSetpointCommand(double newSpeed){
        return Commands.runOnce(() -> setSpeed(newSpeed));
    }

    public Command setPresetCommand(IntakeSpeed preset){
        return Commands.runOnce(() -> intakeMotors(preset));
    }

    @Override
    public void onModeSwitch() {
        intakeMotors(IntakeSpeed.STOP);
    }
}
