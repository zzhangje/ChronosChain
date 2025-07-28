package frc.robot.subsystem.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.Alert;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.interfaces.motor.GenericArmIO;
import frc.lib.interfaces.motor.GenericArmIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericArmIOKraken;
import frc.lib.interfaces.motor.GenericArmIOSim;
import frc.lib.interfaces.motor.GenericRollerIO;
import frc.lib.interfaces.motor.GenericRollerIOInputsAutoLogged;
import frc.lib.interfaces.motor.GenericRollerIOKraken;
import frc.lib.interfaces.motor.GenericRollerIOSim;
import frc.lib.interfaces.sensor.digital.DigitalInputRio;
import frc.robot.Constants.Ports;
import frc.robot.subsystem.intake.IntakeGoal.IntakePivotGoal;
import frc.robot.subsystem.intake.IntakeGoal.IntakeRollerGoal;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  static {
    final var gains = IntakeConfig.getPivotGains();
    IntakeConfig.pivotKp.initDefault(gains.kp());
    IntakeConfig.pivotKd.initDefault(gains.kd());
    IntakeConfig.pivotKs.initDefault(gains.ks());
    IntakeConfig.pivotKg.initDefault(gains.kg());
  }

  @Setter @Getter @AutoLogOutput private IntakePivotGoal pivotGoal = IntakePivotGoal.IDLE;
  @Setter @Getter @AutoLogOutput private IntakeRollerGoal rollerGoal = IntakeRollerGoal.IDLE;

  @Setter private BooleanSupplier hasCoralSupplier;
  private final Debouncer hasCoralDebouncer = new Debouncer(0.2, Debouncer.DebounceType.kFalling);
  @Getter @AutoLogOutput private boolean isAtSetpoint = false;

  private final GenericRollerIO rollerIO;
  private final GenericRollerIO centeringIO;
  private final GenericArmIO pivotIO;

  private final GenericRollerIOInputsAutoLogged rollerIOInputs =
      new GenericRollerIOInputsAutoLogged();
  private final GenericRollerIOInputsAutoLogged centeringIOInputs =
      new GenericRollerIOInputsAutoLogged();
  private final GenericArmIOInputsAutoLogged pivotIOInputs = new GenericArmIOInputsAutoLogged();

  private final Alert rollerOfflineAlert =
      new Alert("Ground intake roller motor offline!", Alert.AlertType.WARNING);
  private final Alert pivotOfflineAlert =
      new Alert("Ground intake pivot motor offline!", Alert.AlertType.WARNING);
  private final Alert centeringOfflineAlert =
      new Alert("Ground intake centering motor offline!", Alert.AlertType.WARNING);

  public double getPivotPositionRad() {
    return pivotIOInputs.positionRad;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerIOInputs);
    Logger.processInputs("Intake/Roller", rollerIOInputs);
    rollerOfflineAlert.set(!rollerIOInputs.connected);

    pivotIO.updateInputs(pivotIOInputs);
    Logger.processInputs("Intake/Pivot", pivotIOInputs);
    pivotOfflineAlert.set(!pivotIOInputs.connected);

    centeringIO.updateInputs(centeringIOInputs);
    Logger.processInputs("Intake/Centering", centeringIOInputs);
    centeringOfflineAlert.set(!centeringIOInputs.connected);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        () ->
            pivotIO.setPdf(
                IntakeConfig.pivotKp.get(),
                IntakeConfig.pivotKd.get(),
                IntakeConfig.pivotKs.get(),
                IntakeConfig.pivotKg.get()),
        IntakeConfig.pivotKp,
        IntakeConfig.pivotKd,
        IntakeConfig.pivotKs,
        IntakeConfig.pivotKg);

    if (rollerGoal != IntakeRollerGoal.EJECT
        && rollerGoal != IntakeRollerGoal.TROUGH
        && hasCoral()) {
      rollerGoal = IntakeRollerGoal.STANDBY;
    }

    pivotIO.setPosition(
        pivotGoal.getAngleRadians(),
        Units.degreesToRadians(IntakeConfig.IntakeMotionMagicVelMeterPerSec.get()),
        Units.degreesToRadians(IntakeConfig.IntakeMotionMagicAccelMeterPerSec2.get()),
        0.0);
    rollerIO.setVoltage(rollerGoal.getRollingVolts());
    centeringIO.setVoltage(rollerGoal.getCenteringVoltage());

    isAtSetpoint =
        Math.abs(pivotIOInputs.positionRad) - pivotGoal.getAngleRadians()
            < IntakeConfig.AT_SETPOINT_THRESHOLD.getAsDouble();
  }

  public Command trough() {
    return Commands.runOnce(
            () -> {
              setPivotGoal(IntakePivotGoal.TROUGH);
              setRollerGoal(IntakeRollerGoal.TROUGH);
            })
        .withName("Intake/Trough");
  }

  public Command inject() {
    return Commands.runOnce(
            () -> {
              setPivotGoal(IntakePivotGoal.DOWN);
              setRollerGoal(IntakeRollerGoal.INJECT);
            })
        .withName("Intake/Inject");
  }

  public Command idle() {
    return Commands.runOnce(
            () -> {
              setPivotGoal(IntakePivotGoal.IDLE);
              setRollerGoal(IntakeRollerGoal.IDLE);
            })
        .withName("Intake/Idle");
  }

  public Command dodge() {
    var cmd =
        Commands.runOnce(() -> setPivotGoal(IntakePivotGoal.DODGE))
            .withName("Intake/Dodge")
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    cmd.addRequirements(this);
    return cmd;
  }

  public void stop() {
    rollerIO.stop();
    pivotIO.stop();
    centeringIO.stop();
  }

  public boolean hasCoral() {
    return hasCoralDebouncer.calculate(hasCoralSupplier.getAsBoolean());
  }

  private Intake(
      GenericRollerIO rollerIO,
      GenericRollerIO centeringIO,
      GenericArmIO pivotIO,
      BooleanSupplier hasCoralSupplier) {
    this.rollerIO = rollerIO;
    this.centeringIO = centeringIO;
    this.pivotIO = pivotIO;
    this.hasCoralSupplier = hasCoralSupplier;
  }

  public static Intake createSim(BooleanSupplier hasCoralSupplier) {
    return new Intake(
        new GenericRollerIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.001),
        new GenericRollerIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.001),
        new GenericArmIOSim(
            DCMotor.getKrakenX60Foc(1),
            IntakeConfig.PIVOT_REDUCTION,
            0.46,
            0.1,
            Units.degreesToRadians(IntakeConfig.MIN_ANGLE_DEGREE),
            Units.degreesToRadians(IntakeConfig.MAX_ANGLE_DEGREE),
            Units.degreesToRadians(IntakeConfig.INIT_ANGLE_DEGREE)),
        hasCoralSupplier);
  }

  public static Intake createReal() {
    return new Intake(
        new GenericRollerIOKraken(
            "IntakeCentering",
            Ports.Can.GROUND_INTAKE_ROLLER,
            IntakeConfig.getCenteringTalonConfig()),
        new GenericRollerIOKraken(
            "IntakeRoller", Ports.Can.GROUND_INTAKE_CENTERING, IntakeConfig.getRollerTalonConfig()),
        new GenericArmIOKraken(
            "IntakePivot",
            Ports.Can.GROUND_INTAKE_PIVOT,
            IntakeConfig.getPivotTalonConfig(),
            Units.degreesToRadians(IntakeConfig.INIT_ANGLE_DEGREE)),
        new DigitalInputRio(Ports.Digital.GROUND_INTAKE_BEAM_BREAK));
  }

  public static Intake createIO() {
    return new Intake(
        new GenericRollerIO() {}, new GenericRollerIO() {}, new GenericArmIO() {}, () -> false);
  }
}
