// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.pathing.robotprofile.Motor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final SwerveModuleState[] kStatesX =
        new SwerveModuleState[] {
          new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
          new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 11;
    public static final int kRearLeftTurningCanId = 13;
    public static final int kFrontRightTurningCanId = 12;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;

    // These values need to be tuned.
    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.5, 0.5, 0.1);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kWeaponsControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera 1
    public static final String kCamera1Name = "AprilTagCamera1";
    public static final Transform3d kRobotToCamera1 =
        new Transform3d(
            Units.inchesToMeters(13),
            Units.inchesToMeters(0),
            Units.inchesToMeters(10),
            new Rotation3d(0, Math.toRadians(-15), 0));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final Matrix<N3, N1> kUntrustworthyStdDevs =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
  }

  public static final class VisionSimConstants {
    public static final int kCameraWidth = 1200;
    public static final int kCameraHeight = 720;
    public static final Rotation2d kDiagonalFOV = Rotation2d.fromDegrees(70);
    public static final double kAvgDetectionNoisePixels = 0.25;
    public static final double kStdDevDetectionNoisePixels = 0.08;

    public static final int kImageCaptureFPS = 20;
    public static final int kAvgLatencyMs = 35;
    public static final int kStdDevLatencyMs = 5;
  }

  public static final class PathingConstants {
    public static final double kRobotMassKg = 63.5;
    public static final double kRobotLengthWidthMeters =
        Units.inchesToMeters(36); // including bumpers. Length and width are the same.
    public static final double kCoralFaceOffset =
        Units.inchesToMeters(13)
            / 2; // page 24: pipes on the same face are 1 ft. 1 in. apart (center to center)
    public static final Motor kDriveMotor = Motor.NEO().gear(4.71);

    public static final Transform2d kTransformLeft =
        new Transform2d(
            kRobotLengthWidthMeters / 2, -kCoralFaceOffset, Rotation2d.fromDegrees(180));
    public static final Transform2d kTransformRight =
        new Transform2d(kRobotLengthWidthMeters / 2, kCoralFaceOffset, Rotation2d.fromDegrees(180));

    public static final double kReefCenterX = Units.inchesToMeters((144.0 + 209.49) / 2);

    // Measurements taken from April Tag coordinates
    // I like doing it as an enum because it makes it easy to organize.
    // Question: does doing it as an enum like this make it less efficient?
    public static enum ReefPose {
      CLOSE(144, 158.5, 180),
      CLOSE_LEFT(160.39, 186.83, 120),
      CLOSE_RIGHT(160.39, 130.17, 240),
      FAR(209.49, 158.5, 0),
      FAR_LEFT(193.1, 186.83, 60),
      FAR_RIGHT(193.1, 130.17, 300);

      private Pose2d leftPose;
      private Pose2d rightPose;

      private ReefPose(double xInches, double yInches, double rotDegrees) {
        Pose2d reefPose =
            new Pose2d(
                Units.inchesToMeters(xInches),
                Units.inchesToMeters(yInches),
                Rotation2d.fromDegrees(rotDegrees));
        this.leftPose = reefPose.transformBy(kTransformLeft);
        this.rightPose = reefPose.transformBy(kTransformRight);
      }

      public Pose2d getPose(boolean right) {
        return right ? rightPose : leftPose;
      }
    }

    public static Pose2d poseFromTag(
        double xInches, double yInches, double rotDegrees, Transform2d transform) {
      return new Pose2d(
              Units.inchesToMeters(xInches),
              Units.inchesToMeters(yInches),
              Rotation2d.fromDegrees(rotDegrees))
          .plus(transform);
    }

    public static final Transform2d kFeederTransform =
        new Transform2d(kRobotLengthWidthMeters / 2, 0.0, new Rotation2d());
    public static final Pose2d kLeftFeederPose = poseFromTag(33.51, 291.20, 306, kFeederTransform);
    public static final Pose2d kRightFeederPose = poseFromTag(33.51, 25.80, 54, kFeederTransform);

    public static final Transform2d kProcessorTransform =
        new Transform2d(kRobotLengthWidthMeters / 2, 0.0, Rotation2d.fromDegrees(0));
    public static final Pose2d kProcessorPose = poseFromTag(235.73, -0.15, 90, kProcessorTransform);
  }
}
