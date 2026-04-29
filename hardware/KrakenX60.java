package frc.robot.VectorKit.hardware;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class KrakenX60 extends TalonFX {
  private final Slot0Configs slot0Configs = new Slot0Configs();
  private final CurrentLimitsConfigs talonCurrentConfigs = new CurrentLimitsConfigs();
  private final MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  private final Supplier<TalonFXSimState> m_simState;
  private final Timer m_simTimer;
  private double lastTime;
  private final DCMotorSim m_sim;

  private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);

  public KrakenX60(int ID) {
    super(ID);

    m_simTimer = new Timer();
    m_simState = () -> super.getSimState();
    m_sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, (18.0 / 70.0)),
            DCMotor.getKrakenX60Foc(1));

    lastTime = m_simTimer.get();
    m_velocityControl.withEnableFOC(false);
  }

  public void logCurrents(String path) {
    Logger.recordOutput(
        String.format("%s/StatorCurrent", path), super.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        String.format("%s/SupplyCurrent", path), super.getSupplyCurrent().getValueAsDouble());
  }

  public void updateSim() {
    m_simState.get().setSupplyVoltage(RobotController.getBatteryVoltage());
    var motorVoltage = m_simState.get().getMotorVoltageMeasure();

    m_sim.setInputVoltage(motorVoltage.in(Volts));
    m_sim.update(m_simTimer.get() - lastTime);
    lastTime = m_simTimer.get();

    m_simState.get().setRawRotorPosition(m_sim.getAngularPosition());
    m_simState.get().setRotorVelocity(m_sim.getAngularVelocity());
  }

  public double getRPM() {
    return RPM.convertFrom(super.getVelocity().getValueAsDouble(), RotationsPerSecond);
  }

  public void setVelocity(double vel, AngularVelocityUnit unit) {
    super.feed();
    super.setControl(m_velocityControl.withVelocity(RotationsPerSecond.convertFrom(vel, unit)));
  }

  public void setInverted(InvertedValue direction) {
    motorOutputConfigs.Inverted = direction;
    super.getConfigurator().apply(motorOutputConfigs);
  }

  public void setBrakeMode(NeutralModeValue mode) {
    motorOutputConfigs.NeutralMode = mode;
    super.getConfigurator().apply(motorOutputConfigs);
  }

  public void setFollower(KrakenX60 follower, MotorAlignmentValue motorAlignment) {
    Follower m_followerRequest = new Follower(super.getDeviceID(), motorAlignment);
    follower.setControl(m_followerRequest);
  }

  public void setPID(double kP, double kI, double kD) {
    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;

    super.getConfigurator().apply(slot0Configs);
  }

  public void setFF(double kS, double kV) {
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    super.getConfigurator().apply(slot0Configs);
  }

  public double getS() {
    return slot0Configs.kS;
  }

  public void setS(double kS) {
    slot0Configs.kS = kS;

    super.getConfigurator().apply(slot0Configs);
  }

  public double getV() {
    return slot0Configs.kV;
  }

  public void setV(double kV) {
    slot0Configs.kV = kV;

    super.getConfigurator().apply(slot0Configs);
  }

  public void setP(double kP) {
    slot0Configs.kP = kP;

    super.getConfigurator().apply(slot0Configs);
  }

  public void setI(double kI) {
    slot0Configs.kI = kI;

    super.getConfigurator().apply(slot0Configs);
  }

  public void setD(double kD) {
    slot0Configs.kD = kD;

    super.getConfigurator().apply(slot0Configs);
  }

  public void updateFromTuner(PidTuner tuner) {
    slot0Configs.kP = tuner.getP();
    slot0Configs.kI = tuner.getI();
    slot0Configs.kD = tuner.getD();
    slot0Configs.kV = tuner.getV();
    slot0Configs.kS = tuner.getS();

    super.getConfigurator().apply(slot0Configs);
  }

  public void setSupplyCurrentLimit(double maxAmps, double minAmps, double seconds) {
    talonCurrentConfigs.withSupplyCurrentLimitEnable(minAmps > 0 && maxAmps > 0);
    talonCurrentConfigs.withSupplyCurrentLimit(maxAmps);
    talonCurrentConfigs.withSupplyCurrentLowerLimit(minAmps);
    talonCurrentConfigs.withSupplyCurrentLowerTime(seconds);

    super.getConfigurator().apply(talonCurrentConfigs);
  }

  public void setStatorCurrentLimit(double amps) {
    talonCurrentConfigs.withStatorCurrentLimitEnable(amps > 0);
    talonCurrentConfigs.withStatorCurrentLimit(amps);

    super.getConfigurator().apply(talonCurrentConfigs);
  }
}
