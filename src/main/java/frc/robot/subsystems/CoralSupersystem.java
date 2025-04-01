// Also coral wheels in this system
// Bigger safe angle (for later)

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorSubConstants;
import frc.robot.Constants.GameConstants.ReefLevels;

@Logged
public class CoralSupersystem {
  /* All of these subsystems should have a command that sets them to each desired position
   * and ends once that position is reached (with another method with "continuous" in its name that
   * allows you to run it without stopping once the goal is reached), and then it should continue to
   * maintain this position after the command stops by setting power in periodic accoding to a saved goal.
   */
  // private final Elevator m_elevator = new Elevator();
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final Wrist m_wrist = new Wrist();
  private final CoralIntake m_coralOmnis = new CoralIntake();
  private final Elevator m_elevator = new Elevator();

  public CoralSupersystem() {
    // m_arm.setDefaultCommand(m_arm.moveToGoalCommand(3));
    m_arm.setDefaultCommand(m_arm.defaultArmCommand(() -> m_wrist.isHorizontal().getAsBoolean()));
    m_coralOmnis.setDefaultCommand(m_coralOmnis.stopCommand());
    m_wrist.setDefaultCommand(m_wrist.toHorizontalContinuous());
    m_elevator.setDefaultCommand(m_elevator.defaultCommand(() -> m_arm.isSafe()));
  }

  public Command armToAngle(Rotation2d angle) {
    return m_arm.moveToGoalCommand(angle);
  }

  /**
   * It can hit things if the wrist is vertical, so we can move the arm first to make sure it's
   * safe.
   */
  public Command moveWristVertical() {
    return m_wrist.toVerticalContinuous();
    // return (m_arm.toHorizontal().andThen(m_wrist.toVerticalContinuous()))
    // .until(() -> m_wrist.isVertical().getAsBoolean());
  }

  public Command coralExtakeOverride() {
    return m_coralOmnis.outakeCommand();
  }

  public Command defaultArm() {
    return (m_wrist.toHorizontalStop().alongWith(m_coralOmnis.stopCommand())).andThen(m_arm.toStored());
  }

  /**
   * If we're moving the wrist horizontal, it won't hit things, but we might want to make sure it's
   * horizontal before moving the arm.
   */
  public Command moveWristHorizontal() {
    return m_wrist.toHorizontalContinuous().until(() -> m_wrist.isHorizontal().getAsBoolean());
  }

  /** Move the arm, wrist, and elevator so that the mechanism is in the stored position. */
  public Command positionStore() {
    return m_wrist
        .toHorizontalStop()
        .andThen(
            m_arm
                .toStored()
                .alongWith(m_coralOmnis.stopCommand())
                .alongWith(m_wrist.toHorizontalContinuous()));
    // return moveWristHorizontal()
    // .andThen(m_arm.toStored().until(() -> m_arm.aboveHorizontal().getAsBoolean()))
    // .andThen(m_elevator.toStored().alongWith(m_arm.toStored()));
  }

  /** Move to intake from the floor. */
  public Command positionFloorIntake() {
    return moveWristHorizontal(); // .andThen(m_elevator.toFloorIntake()).andThen(m_arm.toFloorIntakeStop());
  }

  /** Move to intake from the feeder station. */
  // public Command positionFeederStation() {
  //   return moveWristHorizontal().andThen(m_elevator.toFeeder().alongWith(m_arm.toFeeder()));
  // }

  // /** Position to place coral on any level, including 1. */
  // public Command positionReef(GameConstants.ReefLevels level) {
  //   return
  // moveWristVertical().andThen(m_elevator.toBranch(level).andThen(m_arm.toBranch(level)));
  // }

  public Command floorIntake() {
    return m_arm.toFloorIntakeStop().alongWith(m_coralOmnis.intakeUntilSuccessCommand());
    // return positionFloorIntake().andThen(m_coralOmnis.intakeUntilSuccessCommand());
  }

  public Command floorExtake() {
    return m_arm
        .toFloorIntakeStop()
        .andThen(m_arm.toFloorIntakeStop().alongWith(m_coralOmnis.outakeCommand()));
  }

  public Command placeLevelOne() {
    return m_arm.toL1Stop().andThen(m_arm.toL1().alongWith(m_coralOmnis.outakeCommand()));
  }

  public Command safeArm() {
    return moveWristHorizontal().andThen(m_arm.toStoredSafe());
  }

  public Command armWristBranchPos() {
    return moveWristHorizontal()
        .andThen(
            (m_arm.toBranchStop().raceWith(m_wrist.toHorizontalContinuous()))
                .until(() -> m_arm.safeWrist()))
        .andThen(m_arm.toL2L3().raceWith(m_wrist.toVerticalStop()))
        .andThen(m_arm.toL2L3().alongWith(m_wrist.toVerticalContinuous()));
    // .alongWith(m_coralOmnis.scoreCommand()));
  }

  public Command scoreWheels() {
    return m_coralOmnis.scoreCommand();
  }

  public Command scoreToBranchCommand(ReefLevels level) {
    return safeArm()
        .andThen(m_elevator.toBranchStop(level).raceWith(m_arm.toStored()))
        .andThen(m_elevator.toBranch(level).alongWith(armWristBranchPos()));
    // return
    // safeArm().andThen(m_elevator.toBranch(level).withTimeout(1)).andThen((m_elevator.stayAtBranch(level)).alongWith(armWristL2L3()));
  }

  public Command scoreToBranchCommandStop(ReefLevels level) {
    return safeArm()
        .andThen(m_elevator.toBranchStop(level).raceWith(m_arm.toStored()))
        .andThen(m_elevator.toBranch(level).alongWith(armWristBranchPos()))
        .until(() -> m_elevator.atGoal(ElevatorSubConstants.getGoalForLevel(level)) && m_wrist.isVertical().getAsBoolean());
    // return
    // safeArm().andThen(m_elevator.toBranch(level).withTimeout(1)).andThen((m_elevator.stayAtBranch(level)).alongWith(armWristL2L3()));
  }

  public Command scoreAtBranchCommand(ReefLevels level) {
    return m_elevator.toBranch(level).alongWith(m_arm.toL2L3().alongWith(m_wrist.toVerticalContinuous())).alongWith(m_coralOmnis.scoreCommand());
  }

  public void resetElevator() {
    m_elevator.reset();
  }

  // public Command scoreL2Command() {
  //   return
  // safeArm().andThen(m_elevator.moveToL2Command().withTimeout(1)).andThen(armWristL2L3().alongWith(m_elevator.moveToL2Command()));
  // }

  // public Command scoreL3Command() {
  //   return
  // safeArm().andThen(m_elevator.moveToL3Command().withTimeout(1)).andThen(armWristL2L3().alongWith(m_elevator.moveToL3Command()));
  // }

  // public Command scoreL4Command() {
  //   return
  // safeArm().andThen(m_elevator.moveToL4Command().withTimeout(1)).andThen(armWristL2L3().alongWith(m_elevator.moveToL4Command()));
  // }

  // public Command feederStation() {
  //   return positionFeederStation().andThen(m_coralOmnis.intakeUntilSuccessCommand());
  // }

  // public Command scoreCoral(GameConstants.ReefLevels level) {
  //   return positionReef(level).andThen(m_coralOmnis.scoreCommand());
  // }

  public Command feederIntakeCommand() {
    return ((m_elevator.toFeederCommandStop().alongWith(m_coralOmnis.stopCommand()))
            .raceWith(m_arm.toStored()))
        .andThen(
            m_arm
                .toFeederIntake()
                .alongWith(m_elevator.toFeederCommand())
                .alongWith(m_coralOmnis.intakeUntilSuccessCommand()));
  }

  public CoralIntake getOmnisSubsystem() {
    return m_coralOmnis;
  }

  public Command scoreCommand(ReefLevels level) {
    if (level.equals(ReefLevels.l1)) {
      return placeLevelOne().until(() -> !m_coralOmnis.hasCoral());
    } else {
      return scoreToBranchCommandStop(level)
        .andThen(scoreAtBranchCommand(level).until(() -> m_coralOmnis.hasCoral()))
        .andThen(scoreAtBranchCommand(level).withTimeout(1));
    }
  }
}
