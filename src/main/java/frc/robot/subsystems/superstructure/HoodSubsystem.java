package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants.HoodState;
import frc.robot.utilities.ModeSwitchHandler;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase implements ModeSwitchInterface {
     
    private TalonFX motor;

    private ArmFeedforward FFCalculator;
    private TrapezoidProfile profile;

    private State currentState;
    private Rotation2d currentSetPoint = STARTING_ANGLE;

    DoublePublisher hoodPositionPublisher = NetworkTableInstance.getDefault().getDoubleTopic("hood_position").publish();
    
    public HoodSubsystem() {
        initializeMotor();
        initializeProfile();

        FFCalculator = new ArmFeedforward(S, G, V, A);

        setMechanismAngle(STARTING_ANGLE);

        ModeSwitchHandler.EnableModeSwitchHandler(this);
    }

    //#region Initialization

    private void initializeMotor() {
        motor = new TalonFX(HOOD_MOTOR_ID);
            motor.setNeutralMode(NeutralModeValue.Brake);
        
        TalonFXConfigurator configurator = motor.getConfigurator();
            configurator.apply(PIDConfigs);
            configurator.apply(currentLimits);
            configurator.apply(feedbackConfig);
    }

    private void initializeProfile() {
        profile = new TrapezoidProfile(constraints);
        currentState = new State(STARTING_ANGLE.getRotations(), 0.0);
    }

    //#endregion
    //#region Periodic

    @Override
    public void periodic() {
        currentSetPoint = Rotation2d.fromRotations(
            MathUtil.clamp(
                currentSetPoint.getRotations(), 
                MIN_ANGLE.getRotations(), 
                MAX_ANGLE.getRotations()));

        currentState = profile.calculate(DELTA_TIME, currentState, new State(currentSetPoint.getRotations(), 0.0));

        final PositionVoltage request = new PositionVoltage(currentState.position).withFeedForward(FFCalculator.calculate(currentState.position, currentState.velocity));
            motor.setControl(request);

        hoodPositionPublisher.accept(getPosition().getDegrees());
    }

    //#endregion
    //#region Measurements

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return Rotation2d.fromRotations(position);
    }

    public Rotation2d getTargetPosition(){
        return Rotation2d.fromRotations(currentState.position);
    }
    
    //#endregion
    //#region Movement

    public void resetMechanism(Rotation2d... angleOptional) {
        Rotation2d angle = (angleOptional.length > 0) ? angleOptional[0] : getPosition();
        currentSetPoint = angle;
        currentState = new State(angle.getRotations(), 0.0);
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(angle.getRotations());
        resetMechanism(angle);
    }

    public void setSetpoint(Rotation2d newSetpoint){
        currentSetPoint = newSetpoint;
    }

    public void incrementAngle(Rotation2d delta){
        currentSetPoint = Rotation2d.fromRotations(currentSetPoint.getRotations() + delta.getRotations());
    }

    ////#endregion
    //#region Commands 

    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            if (RobotBase.isReal()) resetMechanism();
        });
    }

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(HoodState preset){
        return setSetpointCommand(preset.angle);
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }

}


