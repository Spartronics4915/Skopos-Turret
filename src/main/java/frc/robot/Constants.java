package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class Constants {

    public static final class HoodConstants {
        
        public static final int HOOD_MOTOR_ID = 15;
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(350);
        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        
        public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = motorConfiguration.Slot0;
                slot0Configs.kS = 0;
                slot0Configs.kV = 0.02;
                slot0Configs.kA = 0.01;
                slot0Configs.kP = 50;
                slot0Configs.kI = 0;
                slot0Configs.kD = 0.15;

            MotionMagicConfigs motionMagicConfigs = motorConfiguration.MotionMagic;
                motionMagicConfigs.MotionMagicCruiseVelocity = 3;
                motionMagicConfigs.MotionMagicExpo_kV = 0.05;
                motionMagicConfigs.MotionMagicExpo_kA = 0.01;
            
            FeedbackConfigs feedbackConfigs = motorConfiguration.Feedback;
                feedbackConfigs.SensorToMechanismRatio = 2.4936523444;
        }

        public enum HoodState {
            PLACE_HOLDER(Rotation2d.fromDegrees(90));

            public Rotation2d angle;

            private HoodState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_ID = 14;
        public static final double MIN_VELOCITY = 0;
        public static final double MAX_VELOCITY = 100;
        public static final double STARTING_VELOCITY = 0;
        
        public static final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        static {
            Slot0Configs slot0Configs = motorConfiguration.Slot0;
                slot0Configs.kS = 0;
                slot0Configs.kV = 0.12;
                slot0Configs.kA = 0;
                slot0Configs.kP = 0.15;
                slot0Configs.kI = 0;
                slot0Configs.kD = 0.01;

            MotionMagicConfigs motionMagicConfigs = motorConfiguration.MotionMagic;
                motionMagicConfigs.MotionMagicAcceleration = 200;
                motionMagicConfigs.MotionMagicJerk = 800;
            
            FeedbackConfigs feedbackConfigs = motorConfiguration.Feedback;
                feedbackConfigs.SensorToMechanismRatio = 0;
        }

        public enum ShooterState {
            PLACE_HOLDER(0);

            public double rpm;

            private ShooterState(double rpm) {
                this.rpm = rpm;
            }
        }
    }

    public static final class IntakeConstants {
        public static final int INTAKE_MOTOR_ID = 22;

        public static final int SMART_CURRENT_LIMIT = 0;
        public static final int SECONDARY_CURRENT_LIMIT = 40;

        public static final double OPEN_LOOP_RAMP_RATE = 0.1;

        public static final EncoderConfig encoderConfig = new EncoderConfig()
            .velocityConversionFactor(0);

        public static final ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig()
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.0001, 0, 0.0);

        public static final SparkBaseConfig motorConfig = new SparkMaxConfig()
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .apply(closedLoopConfig)
            .apply(encoderConfig)
            .openLoopRampRate(OPEN_LOOP_RAMP_RATE)
            .smartCurrentLimit(SMART_CURRENT_LIMIT)
            .secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

        public enum IntakeSpeed {
            STOP(0),
            SLOW(1),
            SHOOT(3);

            public final double intakeSpeed;
            
            private IntakeSpeed(double intakeSpeed) {
                this.intakeSpeed = intakeSpeed;
            }
        }
    }

    public static final class TurretConstants {
        public static final double dt = 0.02;

        public static final int TURRET_MOTOR_ID = 21;
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-270);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(180);

        public static final Constraints constraints = new Constraints(700, 700);

        public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

        static {
            motorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(35)
                .secondaryCurrentLimit(40);
            motorConfig.encoder
                .positionConversionFactor(135.0/7616.0 * 360)
                .velocityConversionFactor(135.0/7616.0 * 360);
            motorConfig
                .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.15, 0, 0); 
        }
    }

    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = 22.475 / 12;
        public static final double WHEEL_BASE = 22.475 / 12;
        public static final double CHASSIS_RADIUS = Math.hypot(TRACK_WIDTH / 2, WHEEL_BASE / 2);

        public static final double MAX_SPEED = Units.feetToMeters(24);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_SPEED * Math.PI / CHASSIS_RADIUS);

        public static Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);
    }

    public static final class IO {
        public static final int DRIVE_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static boolean IS_FIELD_RELATIVE = false;

        public static final double STICK_DEADBAND = 0.1;
        public static final double TRIGGER_DEADBAND = 0.1;

        public static final double HOOD_STEP = 7.5;
        public static final double TURRET_STEP = 1;
    }

    public static final class SuperstructureConstants {
        public static final int INTAKE_LC_ID = 26;
        public static final Distance INTAKE_LC_TRIGGER_DISTANCE = Meters.of(0.109);
        public static final double INTAKE_LC_DEBOUNCE = 0.03;
        public static final double INTAKE_TIMEOUT = 3;
    }

    public static final class VisionConstants {
        public static final Transform3d ROBOT_TO_CAMERA_TAU = new Transform3d(
            new Translation3d(0, 0.028575, 0.576331),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(), 
                Rotation2d.fromDegrees(-15).getRadians(), 
                Rotation2d.fromDegrees(0).getRadians()
            )
        );

        public static final Transform3d ROBOT_TO_CAMERA_ETA = new Transform3d(
            new Translation3d(0.180, -0.353, 0.092079),
            new Rotation3d(
                Rotation2d.fromDegrees(0).getRadians(), 
                Rotation2d.fromDegrees(0).getRadians(), 
                Rotation2d.fromDegrees(0).getRadians()
            )
        );
    
        public static final List<CameraConfig> cameraConfigs = List.of(
            new CameraConfig(
                "Tau", 
                ROBOT_TO_CAMERA_TAU, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)),
            new CameraConfig(
                "Eta",  
                ROBOT_TO_CAMERA_ETA, 
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField))
        );

        public static final SimCameraProperties simCameraProperties = new SimCameraProperties();
            static {
                simCameraProperties.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
                simCameraProperties.setCalibError(0.12, 0.04);
                simCameraProperties.setFPS(60);
                simCameraProperties.setAvgLatencyMs(15);
                simCameraProperties.setLatencyStdDevMs(5);
            }

        public enum VisionState {
            GLOBAL(),
            LOCAL()
        }

        public static final Matrix<N3, N1> baseStdDevs = VecBuilder.fill(0, 0, 0);
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public record CameraConfig(
            String name,
            Transform3d robotToCamera,
            PoseStrategy strategy,
            AprilTagFieldLayout apriltagLayout
        ) {}
    }
    public static final class IntakeTwoConstants {
    
        public static final int INTAKE_TWO_MOTOR_ID = 17;
        public static final boolean INTAKE_MOTOR_INVERTED = false;
        public static final int INTAKE_MOTOR_SMART_LIMIT = 15;
        public static final int INTAKE_MOTOR_SECONDARY_LIMIT = 30;

        public static final double INTAKE_MOTOR_SET_SPEED = 0.4;

        
    }

}   
