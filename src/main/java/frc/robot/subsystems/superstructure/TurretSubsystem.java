package frc.robot.subsystems.superstructure;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants.HoodState;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.ModeSwitchHandler;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

public class TurretSubsystem extends SubsystemBase implements ModeSwitchInterface {
    private SparkMax motor;
    private RelativeEncoder motorEncoder;
    private Rotation2d currentSetPoint = Rotation2d.fromDegrees(0);
    private TrapezoidProfile trapezoidProfile;
    private State currentState = new State(0, 0);
    private SparkClosedLoopController turretClosedLoopController;

    public DoublePublisher turretPosePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Current Pose").publish();
    public DoublePublisher turretDesiredPosePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Turret Desired Pose").publish();
    
    public TurretSubsystem() {
        motor = new SparkMax(TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);

        trapezoidProfile = new TrapezoidProfile(TurretConstants.constraints);
        turretClosedLoopController = motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();

        setMechanismAngle(Rotation2d.fromDegrees(0));
        ModeSwitchHandler.EnableModeSwitchHandler(this);
        motor.configure(TurretConstants.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        currentSetPoint = Rotation2d.fromDegrees(
            MathUtil.clamp(
                currentSetPoint.getDegrees(),
                TurretConstants.MIN_ANGLE.getDegrees(),
                TurretConstants.MAX_ANGLE.getDegrees()
            )
        );

        currentState = trapezoidProfile.calculate(TurretConstants.dt, currentState, new State(currentSetPoint.getDegrees(), 0));
        turretClosedLoopController.setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0);

        turretPosePublisher.accept(getPosition().getDegrees());
        turretDesiredPosePublisher.accept(getTargetPosition().getDegrees());
    }

    public void resetMechanism(Rotation2d... angleOptional) {
        Rotation2d angle = (angleOptional.length > 0) ? angleOptional[0] : getPosition();
        currentSetPoint = angle;
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(motorEncoder.getPosition());
    }

    public Rotation2d getTargetPosition(){
        return currentSetPoint;
    }

    private void setMechanismAngle(Rotation2d angle){
        motorEncoder.setPosition(angle.getDegrees());
        resetMechanism(angle);
    }

    public void setSetpoint(Rotation2d newSetpoint){
        currentSetPoint = newSetpoint;
    }

    public void incrementAngle(Rotation2d delta){
        currentSetPoint = Rotation2d.fromDegrees(currentSetPoint.getDegrees() + delta.getDegrees());
    }

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
