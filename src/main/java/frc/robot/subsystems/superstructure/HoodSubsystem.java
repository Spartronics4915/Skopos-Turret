package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants.HoodState;
import frc.robot.utilities.ModeSwitchHandler;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.HoodConstants.*;

public class HoodSubsystem extends SubsystemBase implements ModeSwitchInterface {

    private TalonFX motor;
    private Rotation2d currentSetPoint = STARTING_ANGLE;

    public DoublePublisher hoodPosePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood_Current_Pose").publish();
    public DoublePublisher hoodDesiredPosePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood_Desired_Pose").publish();
    public DoublePublisher hoodDesiredStatePosePublisher = NetworkTableInstance.getDefault().getDoubleTopic("Hood_Desired_State_Pose").publish();

    public HoodSubsystem() {
        motor = new TalonFX(HOOD_MOTOR_ID);
        motor.getConfigurator().apply(motorConfiguration);
        motor.setNeutralMode(NeutralModeValue.Brake);
        setMechanismAngle(STARTING_ANGLE);
        ModeSwitchHandler.EnableModeSwitchHandler(this);

        SmartDashboard.putData("BOOM!", setSetpointCommand(Rotation2d.fromDegrees(90)));
        SmartDashboard.putData("SPLAT!", setSetpointCommand(Rotation2d.fromDegrees(0)));
        SmartDashboard.putData("POW!", setSetpointCommand(Rotation2d.fromDegrees(300)));
    }

    @Override
    public void periodic() {
        currentSetPoint = fromRotations(
            MathUtil.clamp(
                toRotations(currentSetPoint),
                toRotations(MIN_ANGLE),
                toRotations(MAX_ANGLE)
            )
        );

        final MotionMagicExpoVoltage request = new MotionMagicExpoVoltage(toRotations(currentSetPoint));
        motor.setControl(request);
        hoodPosePublisher.accept(getPosition().getDegrees());
        hoodDesiredPosePublisher.accept(getTargetPosition().getDegrees());
    }

    public void resetMechanism(Rotation2d... angleOptional) {
        Rotation2d angle = (angleOptional.length > 0) ? angleOptional[0] : getPosition();
        currentSetPoint = angle;
    }

    public Rotation2d getPosition() {
        double position = motor.getPosition().getValue().in(Rotations);
        return fromRotations(position);
    }

    public Rotation2d getTargetPosition(){
        return currentSetPoint;
    }

    private void setMechanismAngle(Rotation2d angle){
        motor.setPosition(toRotations(angle));
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

    private static Rotation2d fromRotations(double rotations) {
        return Rotation2d.fromDegrees(rotations * 360.0);
    }

    private static double toRotations(Rotation2d angle) {
        return angle.getDegrees() / 360.0;
    }
}
