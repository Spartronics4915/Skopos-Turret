import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

public class SwerveSubsystem extends SubsystemBase implements ModeSwitchInterface {

    SwerveDriveKinematics kinematics;

    public SwerveSubsystem () {
        kinematics = new SwerveDriveKinematics(
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(12.5)), // Front Left
            new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(-12.5)), // Front Right
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(12.5)), // Back Left
            new Translation2d(Units.inchesToMeters(-12.5), Units.inchesToMeters(-12.5))  // Back Right
        );
    }

    public void drive() {
        // Create test ChassisSpeeds going X = 14in, Y=4in, and spins at 30deg per second.
        ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(14), Units.inchesToMeters(4), Units.degreesToRadians(30));
        
        // Get the SwerveModuleStates for each module given the desired speeds.
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
        // Output order is Front-Left, Front-Right, Back-Left, Back-Right
    }

    ModuelAngleOptimization () {
        var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
        new Rotation2d(m_turningEncoder.getDistance()));
    }

    CosineCompensation () {
        var currentAngle = new Rotation2d.fromRadians(m_turningEncoder.getDistance());
        var frontLeftOptimized = SwerveModuleState.optimize(frontLeft, currentAngle);
        frontLeftOptimized.speedMetersPerSecond *= frontLeftOptimized.angle.minus(currentAngle).getCos();
    }

    FieldOrientedDrive () {
        // The desired field relative speed here is 2 meters per second
        // toward the opponent's alliance station wall, and 2 meters per
        // second toward the left field boundary. The desired rotation
        // is a quarter of a rotation per second counterclockwise. The current
        // robot angle is 45 degrees.
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));
        // Now use this in our kinematics
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    }

    @Override
    public void periodic() {
        // Periodically send a set of module states
        publisher.set(new SwerveModuleState[] {
          frontLeftState,
          frontRightState,
          backLeftState,
          backRightState
        });
      }
}