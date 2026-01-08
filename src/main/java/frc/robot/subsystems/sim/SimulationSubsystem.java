package frc.robot.subsystems.sim;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimulationSubsystem extends SubsystemBase {

    private final double g = 9.81;
    private final double INITIAL_HEIGHT = 0.5;
    private final double WHEEL_RADIUS = Meters.convertFrom(2.5, Inches);
    private final double SHOT_EFFICIENCY = 0.5;

    private double flightTime = 0;
    private Pose3d currentBallPose = new Pose3d();
    private boolean isFlying = false;

    private double vX0, vY0, vZ0;
    private double x0, y0;

    StructPublisher<Pose3d> ballPosePublisher = NetworkTableInstance.getDefault().getStructTopic("Ball", Pose3d.struct).publish();

    public SimulationSubsystem() {}

    @Override
    public void periodic() {
        if (isFlying) {
            updateBallFlightSimulation(flightTime);
            ballPosePublisher.accept(currentBallPose);
        }
    }

    public void startFlightSimulation(double flywheelRpm, double hoodAngleRad, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        // 1. Calculate Ball Exit Speed
        double surfaceSpeed = (flywheelRpm * 2 * Math.PI / 60.0) * WHEEL_RADIUS;
        double vBall = surfaceSpeed * SHOT_EFFICIENCY;

        // 2. Break ball velocity into components relative to robot heading
        double vHorizontal = vBall * Math.cos(hoodAngleRad);
        vZ0 = vBall * Math.sin(hoodAngleRad);

        // 3. Combine with Robot Field-Relative Velocity
        // Transform robot-relative speeds to field-relative speeds
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(robotSpeeds, robotPose.getRotation());
        
        vX0 = (vHorizontal * robotPose.getRotation().getCos()) + fieldSpeeds.vxMetersPerSecond / 20;
        vY0 = (vHorizontal * robotPose.getRotation().getSin()) + fieldSpeeds.vyMetersPerSecond / 20;

        // 4. Set Starting Position
        x0 = robotPose.getX();
        y0 = robotPose.getY();

        System.out.println("Flywheel speed: " + flywheelRpm);
        System.out.println("Hood Angle: " + hoodAngleRad);
        System.out.println("Starting pose: " + x0);
        System.out.println("Field speeds: " + vX0);
        
        flightTime = 0.02;
        isFlying = true;
    }

    public void updateBallFlightSimulation(double deltaTime) {
        flightTime += deltaTime;

        // Standard Projectile Motion Equations
        double x = x0 + vX0 * flightTime;
        double y = y0 + vY0 * flightTime;
        double z = INITIAL_HEIGHT + (vZ0 * flightTime) - (0.00001 * g * Math.pow(flightTime, 2));

        // Update Pose3:
        currentBallPose = new Pose3d(x, y, z, new Rotation3d());

        // Stop simulation if the ball hits the floor
        if (z < 0) {
            isFlying = false;
        }
    }   
}
