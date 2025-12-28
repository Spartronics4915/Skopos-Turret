package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.utilities.ModeSwitchHandler;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

public class ShooterSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private TalonFX motor;
    private double currentSetPoint = STARTING_VELOCITY;

    public DoublePublisher shooterVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood_Current_Pose").publish();
    public DoublePublisher shooterDesiredVelocityPublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood_Desired_Pose").publish();
    
    public ShooterSubsystem() {
        motor = new TalonFX(SHOOTER_MOTOR_ID);
        motor.getConfigurator().apply(motorConfiguration);
        motor.setNeutralMode(NeutralModeValue.Brake);
        setVelocity(STARTING_VELOCITY);
        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    @Override
    public void periodic() {
        currentSetPoint = 
            MathUtil.clamp(
                currentSetPoint,
                MIN_VELOCITY,
                MAX_VELOCITY
            );

        final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(currentSetPoint);
        motor.setControl(request);
    }

    public void resetMechanism(double... velocityOptional) {
        double velocity = (velocityOptional.length > 0) ? velocityOptional[0] : getVelocity();
        currentSetPoint = velocity;
    }

    public double getVelocity() {
        double velocity = motor.getVelocity().getValueAsDouble();
        return velocity;
    }

    public double getTargetVelocity(){
        return currentSetPoint;
    }

    private void setVelocity(double velocity){
        motor.setPosition(velocity);
        resetMechanism(velocity);
    }

    public void setSetpoint(double newSetpoint){
        currentSetPoint = newSetpoint;
    }

    public void incrementVelocity(double velocity){
        currentSetPoint = currentSetPoint + velocity;
    }

    public Command manualMode(double velocity){
        return this.runEnd(() -> {
            incrementVelocity(velocity);
        }, () -> {
            if (RobotBase.isReal()) resetMechanism();
        });
    }

    public Command setSetpointCommand(double newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(ShooterState preset){
        return setSetpointCommand(preset.rpm);
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }
}
