package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ElevatorSubConstants;
import frc.robot.Constants.ModuleConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;
      double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50);
      drivingConfig
          .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(0.04, 0, 0)
          .velocityFF(drivingVelocityFeedForward)
          .outputRange(-1, 1);

      turningConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
      turningConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(true)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0); // radians per second
      turningConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to them for your own robot!
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class Algae {
    public static final SparkMaxConfig wheelsConfig = new SparkMaxConfig();
    public static final SparkMaxConfig armConfig = new SparkMaxConfig();

    static {
      wheelsConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(60);

      // Configure basic settings of the arm motor
      armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(12);
      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      armConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control
          .p(0.1)
          .outputRange(-1, 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(2000)
          .maxAcceleration(10000)
          .allowedClosedLoopError(0.25);

      armConfig
          .absoluteEncoder
          .positionConversionFactor(AlgaeConstants.kConversionFactor)
          .velocityConversionFactor(AlgaeConstants.kConversionFactor);
    }
  }

  public static final class CoralArm {
    public static final double kG = 0;
    public static final SparkMaxConfig coralArmConfig = new SparkMaxConfig();

    static {
      double armFactor = 2 * Math.PI;
      // double armKv = 0.5; // V*s/radian

      coralArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60);

      coralArmConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(false)
          .positionConversionFactor(armFactor) // radians
          .velocityConversionFactor(armFactor / 60.0); // radians per second

      coralArmConfig.signals.absoluteEncoderPositionPeriodMs(20);

      coralArmConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .pid(8, 0, 0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, armFactor);

      coralArmConfig
          .closedLoop
          .maxMotion
          // Set MAXMotion parameters for position control. We don't need to pass
          // a closed loop slot, as it will default to slot 0.
          .maxVelocity(30) // radians/minute
          .maxAcceleration(30) // radians/minute/second
          .allowedClosedLoopError(1);

      // Constants.ElevatorConstants.Controller.Kp,
      //     Constants.ElevatorConstants.Controller.Ki,
      //     Constants.ElevatorConstants.Controller.Kd,
      //     new TrapezoidProfile.Constraints(
      //         ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration)
      // We will need to make a lot more changes to the config.
    }
  }

  public static final class CoralArmSubsystem {
    public static final SparkMaxConfig coralArmConfig = new SparkMaxConfig();

    static {
      coralArmConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true);
      coralArmConfig
          .absoluteEncoder
          .inverted(true)
          .positionConversionFactor(Units.rotationsToRadians(1))
          .velocityConversionFactor(Units.rotationsPerMinuteToRadiansPerSecond(1));
      coralArmConfig.signals.absoluteEncoderPositionPeriodMs(20);
    }
  }

  public static final class Coral {
    public static final SparkMaxConfig wheelsConfig = new SparkMaxConfig();

    static {
      wheelsConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(20);
    }
  }

  public static final class ElevatorConfigs {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorConfig2 = new SparkMaxConfig();

    static {
      // double elevatorPosFactor = 2 * Math.PI * ElevatorConstants.Measurements.kDrumRadius / 5;

      elevatorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
      elevatorConfig.encoder.uvwAverageDepth(4).uvwMeasurementPeriod(8);
      elevatorConfig
          .encoder
          .positionConversionFactor(ElevatorSubConstants.kPosFactor)
          .velocityConversionFactor(ElevatorSubConstants.kVelocityFactor);
      // elevatorConfig.encoder.positionConversionFactor(elevatorPosFactor);
      // elevatorConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      //     // These are example gains you may need to them for your own robot!
      //     .pid(1, 0, 0)
      //     .outputRange(-1, 1);

      elevatorConfig2.idleMode(IdleMode.kBrake).smartCurrentLimit(80);
    }
  }

  public static final class ClimberConfigs {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      climberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);
    }
  }

  public static final class Wrist {
    public static final SparkMaxConfig wristConfig = new SparkMaxConfig();
    public static final double kG = 0.01;

    static {
      double wristFactor = 2 * Math.PI;
      // Configure basic settings of the arm motor
      wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);

      wristConfig
          .absoluteEncoder
          // Invert the turning encoder, since the output shaft rotates in the opposite
          // direction of the steering motor in the MAXSwerve Module.
          .inverted(false)
          .positionConversionFactor(wristFactor) // radians
          .velocityConversionFactor(wristFactor / 60.0); // radians per second

      wristConfig.signals.absoluteEncoderPositionPeriodMs(20);

      // wristConfig
      //     .closedLoop
      //     .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      //     // Set PID values for position control. We don't need to pass a closed
      //     // loop slot, as it will default to slot 0.
      //     .pid(0.4, 0, 0)
      //     .outputRange(-1, 1)
      //     .positionWrappingEnabled(true)
      //     .positionWrappingInputRange(0, wristFactor);

      // wristConfig
      //     .closedLoop
      //     .maxMotion
      //     // Set MAXMotion parameters for position control. We don't need to pass
      //     // a closed loop slot, as it will default to slot 0.
      //     .maxVelocity(30) // radians/minute
      //     .maxAcceleration(30) // radians/minute/second
      //     .allowedClosedLoopError(1);
    }
  }
}
