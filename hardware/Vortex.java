package frc.robot.VectorKit.hardware;

import static com.revrobotics.PersistMode.kNoPersistParameters;
import static com.revrobotics.ResetMode.kNoResetSafeParameters;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.AngularVelocityUnit;

public class Vortex extends SparkFlex {
  private final SparkClosedLoopController pidController;
  private final SparkBaseConfig config;

  public Vortex(int ID) {
    super(ID, MotorType.kBrushless);
    pidController = super.getClosedLoopController();

    config = new SparkFlexConfig();
    super.configAccessor.closedLoop.getP();
    config.closedLoop.pid(0, 0, 0);

    super.configure(config, kNoResetSafeParameters, kNoPersistParameters);
  }

  public void setVelocity(double vel, AngularVelocityUnit unit) {
    pidController.setSetpoint(RPM.convertFrom(vel, unit), ControlType.kVelocity);
  }

  public void setPID(double kP, double kI, double kD) {
    //   super.set
    // slot0Configs.kP = kP;
    // slot0Configs.kI = kI;
    // slot0Configs.kD = kD;

    // super.getConfigurator().apply(slot0Configs);
  }

  // public void setFF(double kS, double kV) {
  //     slot0Configs.kS = kS;
  //     slot0Configs.kV = kV;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public double getS() {
  //     return slot0Configs.kS;
  // }

  // public void setS(double kS) {
  //     slot0Configs.kS = kS;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public double getV() {
  //     return slot0Configs.kV;
  // }

  // public void setV(double kV) {
  //     slot0Configs.kV = kV;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public void setP(double kP) {
  //     slot0Configs.kP = kP;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public void setI(double kI) {
  //     slot0Configs.kI = kI;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public void setD(double kD) {
  //     slot0Configs.kD = kD;

  //     super.getConfigurator().apply(slot0Configs);
  // }

  // public void updateFromTuner(pidTuner tuner) {
  //     slot0Configs.kP = tuner.getP();
  //     slot0Configs.kI = tuner.getI();
  //     slot0Configs.kD = tuner.getD();
  //     slot0Configs.kV = tuner.getV();
  //     slot0Configs.kS = tuner.getS();

  //     super.getConfigurator().apply(slot0Configs);
  // }
}
