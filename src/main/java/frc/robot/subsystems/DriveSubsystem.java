// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.pathing.PathingCommand;
import frc.pathing.PathingCommandGenerator;
import frc.pathing.robotprofile.RobotProfile;
import frc.pathing.utils.AllianceUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PathingConstants;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.photonvision.simulation.VisionSystemSim;

@Logged
public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft =
      new MAXSwerveModule(
          DriveConstants.kFrontLeftDrivingCanId,
          DriveConstants.kFrontLeftTurningCanId,
          DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight =
      new MAXSwerveModule(
          DriveConstants.kFrontRightDrivingCanId,
          DriveConstants.kFrontRightTurningCanId,
          DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft =
      new MAXSwerveModule(
          DriveConstants.kRearLeftDrivingCanId,
          DriveConstants.kRearLeftTurningCanId,
          DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight =
      new MAXSwerveModule(
          DriveConstants.kRearRightDrivingCanId,
          DriveConstants.kRearRightTurningCanId,
          DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
  // private final Vision vision;
  private final SimDeviceSim m_gyroSim = new SimDeviceSim("navX-Sensor", m_gyro.getPort());
  private final SimDouble m_gyroSimAngle = m_gyroSim.getDouble("Yaw");

  final VisionSystemSim visionSim;

  @NotLogged // everything in here is already logged by modules or getPose()
  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d());

  private SwerveModuleState[] m_statesMeasured =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };

  @SuppressWarnings("unused")
  private SwerveModuleState[] m_statesRequested = m_statesMeasured;

  @SuppressWarnings("unused")
  private ChassisSpeeds m_speedsMeasured = new ChassisSpeeds();

  // @SuppressWarnings("unused")
  private ChassisSpeeds m_speedsRequested = new ChassisSpeeds();

  RobotProfile m_robotProfile =
      new RobotProfile(
          PathingConstants.kRobotMassKg,
          ModuleConstants.kWheelDiameterMeters,
          PathingConstants.kRobotLengthWidthMeters,
          PathingConstants.kRobotLengthWidthMeters,
          PathingConstants.kDriveMotor);
  PathingCommandGenerator m_pathGen =
      new PathingCommandGenerator(m_robotProfile, this::getPose, this::driveSpeed, this).withAllianceFlipping(false);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    visionSim = new VisionSystemSim("main-sim");
    visionSim.addAprilTags(VisionConstants.kTagLayout);

    // vision = new Vision(VisionConstants.kCamera1Name, VisionConstants.kRobotToCamera1,
    // visionSim);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // vision
    //     .getEstimatedGlobalPose()
    //     .ifPresent(
    //         est -> {
    //           // Change our trust in the measurement based on the tags we can see
    //           var estStdDevs = vision.getEstimationStdDevs();

    //           addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds,
    // estStdDevs);
    //         });

    m_statesMeasured =
        new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState(),
        };

    m_speedsMeasured = DriveConstants.kDriveKinematics.toChassisSpeeds(m_statesMeasured);
  }

  @Override
  public void simulationPeriodic() {
    double timestep = 20e-3;
    m_frontLeft.simulationPeriodic(timestep);
    m_frontRight.simulationPeriodic(timestep);
    m_rearLeft.simulationPeriodic(timestep);
    m_rearRight.simulationPeriodic(timestep);
    double dTheta = (m_speedsRequested.omegaRadiansPerSecond * timestep) * 180 / Math.PI;
    m_gyroSimAngle.set(m_gyroSimAngle.get() - dTheta);

    visionSim.update(getPose());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
      Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    m_poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered, ySpeedDelivered, rotDelivered, getPose().getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  public void driveSpeed(ChassisSpeeds speeds) {
    drive(
        speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
        speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
        true);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    setModuleStates(DriveConstants.kStatesX);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
    m_statesRequested = desiredStates;
    m_speedsRequested = DriveConstants.kDriveKinematics.toChassisSpeeds(desiredStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Zeroes the heading of the robot and sets the pose.
   *
   * @param pose The pose that the robot will have after reset.
   */
  public void zeroHeading(Pose2d pose) {
    m_gyro.reset();
    resetOdometry(pose);
  }

  /**
   * Returns the current Yaw value in degrees reported by the sensor.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getYaw() {
    return m_gyro.getYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return The robot's heading in degrees with continuous angle. Use when wraparound could be a
   *     problem.
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Command for driving the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public Command driveCommand(
      DoubleSupplier xSpeed,
      DoubleSupplier ySpeed,
      DoubleSupplier rot,
      BooleanSupplier fieldRelative) {
    return run(
        () ->
            drive(
                xSpeed.getAsDouble(),
                ySpeed.getAsDouble(),
                rot.getAsDouble(),
                fieldRelative.getAsBoolean()));
  }

  public Command stopCommand() {
    return driveCommand(() -> 0.0, () -> 0.0, () -> 0.0, () -> true);
  }

  public PathingCommandGenerator getPathingCommandGenerator() {
    return m_pathGen;
  }

  public PathingCommand getToReefPoseCommand(PathingConstants.ReefPose reefPose, boolean right) {
    return m_pathGen.generateToPoseCommand(reefPose.getPose(right));
  }

  // public PathingCommand getToNearestReefCommand(boolean right) {
  //   Pose2d close = PathingConstants.ReefPose.CLOSE.getPose(right);
  //   Pose2d closeL = PathingConstants.ReefPose.CLOSE_LEFT.getPose(right);
  //   Pose2d closeR = PathingConstants.ReefPose.CLOSE_RIGHT.getPose(right);
  //   Pose2d far = PathingConstants.ReefPose.FAR.getPose(right);
  //   Pose2d farL = PathingConstants.ReefPose.FAR_LEFT.getPose(right);
  //   Pose2d farR = PathingConstants.ReefPose.FAR_RIGHT.getPose(right);
  //   return m_pathGen.generateToPoseSupplierCommand(
  //       () -> {
  //         Pose2d robotAt = AllianceUtil.getBluePose();
  //         if (robotAt.getX() < PathingConstants.kReefCenterX) {
  //           // Robot is at a close pose.
  //           return robotAt.nearest(List.of(close, closeL, closeR));
  //         } else {
  //           // Robot is at a far pose.
  //           return robotAt.nearest(List.of(far, farL, farR));
  //         }
  //       });
  // }

  // public PathingCommand getToFeederCommand(boolean right) {
  //   return m_pathGen.generateToPoseCommand(
  //       right ? PathingConstants.kRightFeederPose : PathingConstants.kLeftFeederPose);
  // }

  // public PathingCommand getToProcessorCommand() {
  //   return m_pathGen.generateToPoseCommand(PathingConstants.kProcessorPose);
  // }

  /** Command to set the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return run(this::setX);
  }
}
