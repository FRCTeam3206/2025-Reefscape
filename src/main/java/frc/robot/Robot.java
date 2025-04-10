// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.pathing.utils.AllianceUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GameConstants.ReefLevels;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PathingConstants.NumCoralAuton;
import frc.robot.Constants.PathingConstants.ReefPose;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSupersystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lights;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> m_autonChooser = new SendableChooser<Command>();
  private SendableChooser<Boolean> m_testMode = new SendableChooser<Boolean>();
  private SendableChooser<Boolean> m_rightPickup = new SendableChooser<Boolean>();
  private SendableChooser<NumCoralAuton> m_numCoral = new SendableChooser<NumCoralAuton>();

  private SendableChooser<ReefPose> m_1Pose = new SendableChooser<ReefPose>();
  private SendableChooser<Boolean> m_1Right = new SendableChooser<Boolean>();
  private SendableChooser<ReefLevels> m_1Level = new SendableChooser<ReefLevels>();

  private SendableChooser<ReefPose> m_2Pose = new SendableChooser<ReefPose>();
  private SendableChooser<Boolean> m_2Right = new SendableChooser<Boolean>();
  private SendableChooser<ReefLevels> m_2Level = new SendableChooser<ReefLevels>();

  private SendableChooser<ReefPose> m_3Pose = new SendableChooser<ReefPose>();
  private SendableChooser<Boolean> m_3Right = new SendableChooser<Boolean>();
  private SendableChooser<ReefLevels> m_3Level = new SendableChooser<ReefLevels>();

  private SendableChooser<ReefPose> m_4Pose = new SendableChooser<ReefPose>();
  private SendableChooser<Boolean> m_4Right = new SendableChooser<Boolean>();
  private SendableChooser<ReefLevels> m_4Level = new SendableChooser<ReefLevels>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Algae m_algae = new Algae();
  private final CoralSupersystem m_coral = new CoralSupersystem();

  private final ClimberSubsystem m_climber = new ClimberSubsystem();
  private final Lights m_lights = new Lights();

  private boolean m_fieldRelative = true;
  private boolean m_invertControls = true;
  private double m_speedMultiplier = DriveConstants.kSlowSpeed;

  private double m_lastTime = 0;
  private double m_loopTime = 0;

  private boolean m_fastMode = false;

  @NotLogged private Alliance m_prevAlliance = null;

  // The driver's controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  CommandXboxController m_weaponsController =
      new CommandXboxController(OIConstants.kWeaponsControllerPort);

  public Robot() {
    AllianceUtil.setCustomFieldDesignType(false);

    if (isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    m_lastTime = Timer.getFPGATimestamp();

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    Epilogue.bind(this);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    configureButtonBindings();
    configureDefaultCommands();
    autons();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // if (Robot.isReal()) {
    m_driverController
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_fastMode = !m_fastMode;
                  m_speedMultiplier =
                      m_fastMode ? DriveConstants.kFastSpeed : DriveConstants.kSlowSpeed;
                }));
    m_driverController.button(7).whileTrue(m_robotDrive.setXCommand());
    // m_weaponsController.back().onTrue(new InstantCommand(() -> m_fieldRelative =
    // !m_fieldRelative));
    // m_weaponsController
    //     .a()
    //     .onTrue(m_robotDrive.runOnce(() -> m_robotDrive.zeroHeading(m_robotDrive.getPose())));
    m_driverController.button(2).onTrue(new InstantCommand(() -> resetRobotToFieldCenter()));
    // } else {
    //   m_weaponsController.start().onTrue(new InstantCommand(() -> resetRobotToFieldCenter()));
    // }

    m_driverController.button(3).whileTrue(m_robotDrive.getToNearestReefCommand(false));
    m_driverController.button(4).whileTrue(m_robotDrive.getToNearestReefCommand(true));

    // m_weaponsController.a().whileTrue(m_coral.armToAngle(Rotation2d.fromDegrees(0)));
    // m_weaponsController.x().whileTrue(m_coral.armToAngle(Rotation2d.fromDegrees(45)));
    // m_weaponsController.y().whileTrue(m_coral.armToAngle(Rotation2d.fromDegrees(75)));
    // m_weaponsController.b().whileTrue(m_coral.armToAngle(Rotation2d.fromDegrees(30)));

    m_weaponsController.povUp().whileTrue(m_algae.extendCommandContinuous());
    m_weaponsController.povDown().whileTrue(m_algae.retractCommandContinuous());
    m_weaponsController.rightTrigger().whileTrue(m_algae.intakeCommand());
    m_weaponsController.leftTrigger().whileTrue(m_algae.extakeCommand());

    m_weaponsController.a().whileTrue(m_coral.floorIntake());
    m_weaponsController.b().whileTrue(m_coral.feederIntakeCommand());
    m_weaponsController.povLeft().whileTrue(m_coral.placeLevelOne());
    m_weaponsController.povRight().whileTrue(m_coral.scoreToBranchCommand(ReefLevels.l2));
    m_weaponsController.x().whileTrue(m_coral.scoreToBranchCommand(ReefLevels.l3));
    m_weaponsController.y().whileTrue(m_coral.scoreToBranchCommand(ReefLevels.l4));
    // m_weaponsController.povRight().whileTrue(m_coral.armWristL2L3());
    m_weaponsController.back().whileTrue(m_coral.coralExtakeOverride());

    m_weaponsController.start().whileTrue(m_coral.scoreWheels());

    m_weaponsController.leftBumper().whileTrue(m_climber.deployCommand());
    m_weaponsController.rightBumper().onFalse(m_climber.climbCommand());

    // m_weaponsController.leftBumper().whileTrue(m_robotDrive.getToGoal(PathingConstants.kCenterStartPose));//m_robotDrive.getToReefPoseCommand(ReefPose.CLOSE_RIGHT, true));

    // m_weaponsController.povRight().whileTrue(m_elevator.moveToL2Command());
    // m_weaponsController.y().whileTrue(m_elevator.moveToL4Command());

    // m_weaponsController.b().whileTrue(m_coral.moveWristVertical());

    // m_weaponsController.rightTrigger().whileTrue(m_robotDrive.getToNearestReefCommand(true));
    // m_weaponsController.leftTrigger().whileTrue(m_robotDrive.getToNearestReefCommand(false));
    // m_driverController.b().whileTrue(m_robotDrive.getToReefPoseCommand(ReefPose.CLOSE, true));
  }

  /** Use this method to define default commands for subsystems. */
  private void configureDefaultCommands() {
    if (Robot.isReal()) {
      m_robotDrive.setDefaultCommand(
          m_robotDrive.driveCommand(
              adjustJoystick(
                  m_driverController::getY,
                  () -> m_speedMultiplier,
                  () -> m_invertControls || !m_fieldRelative),
              adjustJoystick(
                  m_driverController::getX,
                  () -> m_speedMultiplier,
                  () -> m_invertControls || !m_fieldRelative),
              adjustJoystick(m_driverController::getTwist, () -> m_speedMultiplier, () -> true),
              () -> m_fieldRelative));
    } else {
      m_robotDrive.setDefaultCommand(
          m_robotDrive.driveCommand(
              adjustJoystick(
                  m_weaponsController::getLeftY,
                  () -> m_speedMultiplier,
                  () -> m_invertControls || !m_fieldRelative),
              adjustJoystick(
                  m_weaponsController::getLeftX,
                  () -> m_speedMultiplier,
                  () -> m_invertControls || !m_fieldRelative),
              adjustJoystick(m_weaponsController::getRightX, () -> m_speedMultiplier, () -> true),
              () -> m_fieldRelative));
    }

    // m_coral.getArm().setDefaultCommand(m_coral.getArm().setVoltageDirectly(() ->
    // m_weaponsController.getLeftY()));

    m_algae.setDefaultCommand(m_algae.holdPositionCommand());
    // m_elevator.setDefaultCommand(m_elevator.stopCommand());

    m_climber.setDefaultCommand(
        m_climber.directControl(
            () -> -MathUtil.applyDeadband(m_weaponsController.getRightY(), 0.5)));

    m_lights.setDefaultCommand(
        m_lights.setPatternCommand(
            () -> {
              if (m_climber.getCanClimb()) {
                return LightsConstants.kClimbGreen; // Light green (Light green-blue)
              } else if (m_robotDrive.autoAligned()) {
                return LightsConstants.kAlignedGreen; // Dark green (Green)
              } else if (m_coral.hasCoral()) {
                return LightsConstants.kCoralRed; // Red-orange
              } else {
                return LightsConstants.kDefaultBlue; // Blue
              }
            },
            () -> {
              return m_climber.getClimbed();
            }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (m_testMode.getSelected()) {
      Command coral1 =
          scoreCoralCommand(m_1Pose.getSelected(), m_1Right.getSelected(), m_1Level.getSelected());
      Command coral2 =
          scoreCoralCommand(m_2Pose.getSelected(), m_2Right.getSelected(), m_2Level.getSelected());
      Command coral3 =
          scoreCoralCommand(m_3Pose.getSelected(), m_3Right.getSelected(), m_3Level.getSelected());
      Command coral4 =
          scoreCoralCommand(m_4Pose.getSelected(), m_4Right.getSelected(), m_4Level.getSelected());
      switch (m_numCoral.getSelected()) {
        case k1Coral:
          return generateAuton(m_rightPickup.getSelected(), coral1);
        case k2Coral:
          return generateAuton(m_rightPickup.getSelected(), coral1, coral2);
        case k3Coral:
          return generateAuton(m_rightPickup.getSelected(), coral1, coral2, coral3);
        case k4Coral:
          return generateAuton(m_rightPickup.getSelected(), coral1, coral2, coral3, coral4);
        default:
          return m_robotDrive.stopCommand();
      }
    } else {
      if (m_autonChooser.getSelected() == null) {
        return m_robotDrive.stopCommand();
      }
      return m_autonChooser.getSelected();
    }
  }

  public void autons() {
    m_autonChooser.setDefaultOption("Nothing", m_robotDrive.stopCommand());
    m_autonChooser.addOption("Basic Forward", simpleForward());
    m_autonChooser.addOption(
        "Score Coral L4", scoreCoralCommand(ReefPose.CLOSE_RIGHT, true, ReefLevels.l4));

    if (Robot.isSimulation()) {
      m_autonChooser.addOption("Feeder Right", pickupCoralCommand(true));
      m_autonChooser.addOption("Feeder Left", pickupCoralCommand(false));
    }
    // generateAuton(false, scoreCoralCommand(ReefPose.CLOSE_RIGHT, true,
    // ReefLevels.l4)));

    // m_autonChooser.addOption(
    //     "1 coral (start center)",
    //     simpleAutonGenerator(PathingConstants.kCenterStartPose, ReefPose.FAR));
    // m_autonChooser.addOption(
    //     "1 coral (start left)",
    //     simpleAutonGenerator(PathingConstants.kLeftStartPose, ReefPose.FAR_LEFT));
    // m_autonChooser.addOption(
    //     "1 coral (start right)",
    //     simpleAutonGenerator(PathingConstants.kRightStartPose, ReefPose.FAR_RIGHT));
    // m_autonChooser.addOption(
    //     "Coral on the left",
    //     generateAuton(
    //         false,
    //         scoreCoralCommand(ReefPose.FAR_LEFT, false, 4),
    //         scoreCoralCommand(ReefPose.FAR_LEFT, true, 4),
    //         scoreCoralCommand(ReefPose.CLOSE_LEFT, false, 4),
    //         scoreCoralCommand(ReefPose.CLOSE_LEFT, true, 4),
    //         scoreCoralCommand(ReefPose.CLOSE, false, 4)));
    // m_autonChooser.addOption("Processor", m_robotDrive.getToProcessorCommand());

    m_testMode.setDefaultOption("Normal Mode", false);
    m_testMode.addOption("Test Mode", true);

    m_numCoral.setDefaultOption("One Coral", NumCoralAuton.k1Coral);
    m_numCoral.addOption("Two Coral", NumCoralAuton.k2Coral);
    m_numCoral.addOption("Three Coral", NumCoralAuton.k3Coral);
    m_numCoral.addOption("Four Coral", NumCoralAuton.k4Coral);

    m_rightPickup.setDefaultOption("Right Pickup", true);
    m_rightPickup.addOption("Left Pickup", false);

    m_1Level.setDefaultOption("Level 1", ReefLevels.l1);
    m_1Level.addOption("Level 2", ReefLevels.l2);
    m_1Level.addOption("Level 3", ReefLevels.l3);
    m_1Level.addOption("Level 4", ReefLevels.l4);

    m_1Pose.setDefaultOption("Far Pose", ReefPose.FAR);
    m_1Pose.addOption("Far Left Pose", ReefPose.FAR_LEFT);
    m_1Pose.addOption("Far Right Pose", ReefPose.FAR_RIGHT);
    m_1Pose.addOption("Close Pose", ReefPose.CLOSE);
    m_1Pose.addOption("Close Left Pose", ReefPose.CLOSE_LEFT);
    m_1Pose.addOption("Close Right Pose", ReefPose.CLOSE_RIGHT);

    m_1Right.setDefaultOption("Right", true);
    m_1Right.addOption("Left", false);

    m_2Level.setDefaultOption("Level 1", ReefLevels.l1);
    m_2Level.addOption("Level 2", ReefLevels.l2);
    m_2Level.addOption("Level 3", ReefLevels.l3);
    m_2Level.addOption("Level 4", ReefLevels.l4);

    m_2Pose.setDefaultOption("Far Pose", ReefPose.FAR);
    m_2Pose.addOption("Far Left Pose", ReefPose.FAR_LEFT);
    m_2Pose.addOption("Far Right Pose", ReefPose.FAR_RIGHT);
    m_2Pose.addOption("Close Pose", ReefPose.CLOSE);
    m_2Pose.addOption("Close Left Pose", ReefPose.CLOSE_LEFT);
    m_2Pose.addOption("Close Right Pose", ReefPose.CLOSE_RIGHT);

    m_2Right.setDefaultOption("Right", true);
    m_2Right.addOption("Left", false);

    m_3Level.setDefaultOption("Level 1", ReefLevels.l1);
    m_3Level.addOption("Level 2", ReefLevels.l2);
    m_3Level.addOption("Level 3", ReefLevels.l3);
    m_3Level.addOption("Level 4", ReefLevels.l4);

    m_3Pose.setDefaultOption("Far Pose", ReefPose.FAR);
    m_3Pose.addOption("Far Left Pose", ReefPose.FAR_LEFT);
    m_3Pose.addOption("Far Right Pose", ReefPose.FAR_RIGHT);
    m_3Pose.addOption("Close Pose", ReefPose.CLOSE);
    m_3Pose.addOption("Close Left Pose", ReefPose.CLOSE_LEFT);
    m_3Pose.addOption("Close Right Pose", ReefPose.CLOSE_RIGHT);

    m_3Right.setDefaultOption("Right", true);
    m_3Right.addOption("Left", false);

    m_4Level.setDefaultOption("Level 1", ReefLevels.l1);
    m_4Level.addOption("Level 2", ReefLevels.l2);
    m_4Level.addOption("Level 3", ReefLevels.l3);
    m_4Level.addOption("Level 4", ReefLevels.l4);

    m_4Pose.setDefaultOption("Far Pose", ReefPose.FAR);
    m_4Pose.addOption("Far Left Pose", ReefPose.FAR_LEFT);
    m_4Pose.addOption("Far Right Pose", ReefPose.FAR_RIGHT);
    m_4Pose.addOption("Close Pose", ReefPose.CLOSE);
    m_4Pose.addOption("Close Left Pose", ReefPose.CLOSE_LEFT);
    m_4Pose.addOption("Close Right Pose", ReefPose.CLOSE_RIGHT);

    m_4Right.setDefaultOption("Right", true);
    m_4Right.addOption("Left", false);

    SmartDashboard.putData("Auton Chooser", m_autonChooser);
    SmartDashboard.putData("Test Mode", m_testMode);
    SmartDashboard.putData("Coral Num", m_numCoral);
    SmartDashboard.putData("Right Pickup", m_rightPickup);

    SmartDashboard.putData("1 Level", m_1Level);
    SmartDashboard.putData("1 Pose", m_1Pose);
    SmartDashboard.putData("1 Right", m_1Right);

    SmartDashboard.putData("2 Level", m_2Level);
    SmartDashboard.putData("2 Pose", m_2Pose);
    SmartDashboard.putData("2 Right", m_2Right);

    SmartDashboard.putData("3 Level", m_3Level);
    SmartDashboard.putData("3 Pose", m_3Pose);
    SmartDashboard.putData("3 Right", m_3Right);

    SmartDashboard.putData("4 Level", m_4Level);
    SmartDashboard.putData("4 Pose", m_4Pose);
    SmartDashboard.putData("4 Right", m_4Right);
  }

  public Command simpleForward() {
    return m_robotDrive.driveCommand(() -> 0.3, () -> 0.0, () -> 0.0, () -> false).withTimeout(0.5);
  }

  public Command simpleAutonGenerator(Pose2d startPose, ReefPose reefPose) {
    return new InstantCommand(() -> m_robotDrive.resetOdometry(startPose))
        .andThen(simpleForward())
        .andThen(m_robotDrive.getToReefPoseCommand(reefPose, true))
        .andThen(m_robotDrive.stopOnceCommand())
        .andThen(m_coral.placeLevelOne().withTimeout(4));
  }

  public Command generateAuton(boolean right, Command... scoreCoralCommands) {
    Command auton = robotForwardCommand().andThen(scoreCoralCommands[0]);
    for (int i = 1; i < scoreCoralCommands.length; i++) {
      auton = auton.andThen(pickupCoralCommand(right)).andThen(scoreCoralCommands[i]);
    }
    return auton;
  }

  /**
   * We can run this at the begining of any autonomous routine so that we first drive forward, to
   * make sure we don't hit the cages.
   */
  public Command robotForwardCommand() {
    return m_robotDrive.driveCommand(() -> 0.5, () -> 0.0, () -> 0.0, () -> false).withTimeout(.25);
  }

  /**
   * Pick up coral from the feeder station
   *
   * @param right Whether to pick up from the right or left feeder station (from driver view, since
   *     we have a rotated field).
   */
  public Command pickupCoralCommand(boolean right) {
    return m_robotDrive
        .getToFeederCommand(right)
        .andThen(m_coral.feederIntakeCommand().raceWith(m_robotDrive.setXAlignedCommand()));
  }

  /**
   * Score coral on the reef.
   *
   * @param reefPose Which location on the reef to score it on.
   * @param right Whether it should be the right side on that face (from the robot's perspective).
   * @param level Which level to score the coral on.
   * @return A Command to score coral on the reef.
   */
  public Command scoreCoralCommand(ReefPose reefPose, boolean right, ReefLevels level) {
    return (m_robotDrive.getToReefPoseCommand(reefPose, right).raceWith(m_coral.defaultArm()))
        .andThen(m_robotDrive.setXAlignedCommand().raceWith(m_coral.scoreCommand(level)));
  }

  /**
   * Apply desired adjustments to a joystick input, such as deadbanding and nonlinear transforms.
   *
   * @param input The input value from the joystick
   * @param negate Whether to invert the input
   * @return The adjusted value from the joystick
   */
  private DoubleSupplier adjustJoystick(
      DoubleSupplier input, DoubleSupplier multiplier, BooleanSupplier negate) {
    return () -> {
      double value = input.getAsDouble();
      if (negate.getAsBoolean()) {
        value = -value;
      }
      value = MathUtil.applyDeadband(value, OIConstants.kDriveDeadband);
      value = multiplier.getAsDouble() * value;
      return value;
    };
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double now = Timer.getFPGATimestamp();
    m_loopTime = now - m_lastTime;
    m_lastTime = now;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    // AllianceUtil.setAlliance();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_coral.resetElevator();
    m_climber.resetClimb();

    if (!DriverStation.getAlliance().isEmpty()) {
      var alliance = DriverStation.getAlliance().get();
      m_invertControls = alliance.equals(Alliance.Blue);
      if (m_prevAlliance == null || !m_prevAlliance.equals(alliance)) {
        resetRobotToFieldCenter();
        m_prevAlliance = alliance;
      }
    }
    if (Robot.isSimulation()) {
      m_invertControls = true;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  public double getLoopTime() {
    return m_loopTime;
  }

  public void resetRobotToFieldCenter() {
    var field = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    var heading =
        (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red)
            ? 180.0
            : 0.0;
    m_robotDrive.zeroHeading();
    m_robotDrive.resetOdometry(
        new Pose2d(
            m_robotDrive.getPose().getX(), // field.getFieldLength() / 2,
            m_robotDrive.getPose().getY(), // field.getFieldWidth() / 2,
            Rotation2d.fromDegrees(heading)));
  }
}
