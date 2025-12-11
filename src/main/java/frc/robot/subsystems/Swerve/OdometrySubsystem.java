import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants.HoodState;
import frc.robot.utilities.ModeSwitchHandler;
import frc.robot.utilities.ModeSwitchHandler.ModeSwitchInterface;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.HoodConstants.*;

public class OdometrySubsystem extends SwerveSubsystem implements ModeSwitchInterface {
    
    OdometrySubsystem m_odometry = new SwerveDriveOdometry(
    m_kinematics, m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
        m_frontLeftModule.getPosition(),
        m_frontRightModule.getPosition(),
        m_backLeftModule.getPosition(),
        m_backRightModule.getPosition()
    }, new Pose2d(0.0, 0.0, new Rotation2d()));

    public OdometrySubsystem () {
        
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
    }

    @Override
    public void periodic () {
        // Get the rotation of the robot from the gyro.
        var gyroAngle = m_gyro.getRotation2d();
        // Update the pose
         m_pose = m_odometry.update(gyroAngle,
         new SwerveModulePosition[] {
            m_frontLeftModule.getPosition(), m_frontRightModule.getPosition(),
            m_backLeftModule.getPosition(), m_backRightModule.getPosition()
        });
    }
}
