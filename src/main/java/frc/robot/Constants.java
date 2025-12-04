// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//TODO: Add actuall values for almost all of these constants

public final class Constants {
    
    public static final class ShooterConstants {
    
        public static final int SHOOTER_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum ShooterState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private ShooterState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class IntakeConstants {
    
        public static final int INTAKE_MOTOR_ID = 0;
        
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
    
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_LIMIT);
    
        public static final double SENSOR_MECHANISM_RATIO = 0;

        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);
        
            public enum IntakeState {
            PLACE_HOLDER(0);
            
            public final double speed;
        
            private IntakeState(double speed) {
                this.speed = speed;
            }
        }
    }

    public static final class HoodConstants {
        
        public static final int HOOD_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum HoodState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private HoodState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class TurretConstants {
        
        public static final int TURRET_MOTOR_ID = 0;
            
        public static final double P = 0;
        public static final double I = 0;
        public static final double D = 0;
        
        public static final SlotConfigs PIDConfigs = new SlotConfigs()
            .withKP(P)
            .withKI(I)
            .withKD(D);
        
        public static final boolean SUPPLY_CURRENT_LIMIT_ENABLE = true;
        public static final double SUPPLY_CURRENT_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_LIMIT = 0;
        public static final double SUPPLY_CURRENT_LOWER_TIME = 1;

        public static final CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(SUPPLY_CURRENT_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SUPPLY_CURRENT_LOWER_LIMIT)
            .withSupplyCurrentLowerTime(SUPPLY_CURRENT_LOWER_TIME);

        public static final double SENSOR_MECHANISM_RATIO = 0;
        
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs()
            .withSensorToMechanismRatio(SENSOR_MECHANISM_RATIO);

        public static final double MAX_VELOCITY = 0;
        public static final double MAX_ACCELERATION = 0;

        public static final Constraints constraints = new Constraints(MAX_VELOCITY, MAX_ACCELERATION);

        public static final Rotation2d STARTING_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        public static final double STARTING_VELOCITY = 0;
        
        public static final double DELTA_TIME = 0.02;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public enum TurretState {
            PLACE_HOLDER(Rotation2d.fromDegrees(0));

            public Rotation2d angle;

            private TurretState(Rotation2d angle) {
                this.angle = angle;
            }
        }
    }

    public static final class Swerve Constants {
        // TO BE FILLED

        //Locations for the wheels of the Swerve Drive
        Translation2d m_frontLeftLocation = new Translation2d(0.0, 0.0);
        Translation2d m_frontRightLocation = new Translation2d(0.0, -0.0);
        Translation2d m_backLeftLocation = new Translation2d(-0.0, 0.0);
        Translation2d m_backRightLocation = new Translation2d(-0.0, -0.0);

        //Location of the center of roation (center of robot)
        Translation2d m_centerOfRotation = new Translation2d(0.0, 0.0);

        //Measurments of the robot
        public static final double kTrackWidth = Units.inchesToMeters(22.475);
        public static final double kWheelbase = Units.inchesToMeters(22.475);
        public static final double kChassisRadius = Math.hypot(
                kTrackWidth / 2, kWheelbase / 2);

        //Max speeds of wheels and turning
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(5);
        public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) * Math.PI / kChassisRadius);
    }
}   
