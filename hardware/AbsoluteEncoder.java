package frc.robot.VectorKit.hardware;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import java.util.Random;
import java.util.function.Supplier;

public class AbsoluteEncoder extends DutyCycleEncoder {
  private final double kOffset;
  private final DutyCycleEncoderSim m_sim;
  private double simState;

  private Supplier<Double> m_sourcePosition;
  private double kGearRatio;

  private boolean kNoiseEnabled = false;
  private Random kNoise = new Random();

  private boolean kIsReversed = false;

  public AbsoluteEncoder(int channel, double offset) {
    super(channel);
    kOffset = (offset / 360.0);

    if (Utils.isSimulation()) {
      m_sim = new DutyCycleEncoderSim(channel);
      m_sim.set(0.0);
      kNoiseEnabled = true;
    } else {
      m_sim = null;
    }
  }

  public void attachSource(Supplier<Double> source, int sourceGearTeeth, int encoderGearTeeth) {
    m_sourcePosition = source;
    kGearRatio = (double) sourceGearTeeth / (double) encoderGearTeeth;
  }

  public void attachSource(
      Supplier<Double> source, double sourceGearTeeth, double encoderGearTeeth) {
    m_sourcePosition = source;
    kGearRatio = sourceGearTeeth / encoderGearTeeth;
  }

  public void setReversed(boolean reversed) {
    kIsReversed = reversed;
  }

  public double getRaw() {
    return super.get();
  }

  // TODO: Use simulated state as a sanity check
  @Override
  public double get() {
    if (Utils.isSimulation()) return getSim();
    double degrees = getRaw() - kOffset;
    if (degrees < 0.0) degrees += 1.0;
    if (degrees > 1.0) degrees -= 1.0;
    if (kIsReversed) degrees = 1.0 - degrees;
    return degrees * 360.0;
  }

  public double get(double min, double max) {
    if (Utils.isSimulation()) return getSim(min, max);
    double degrees = getRaw() - kOffset;
    if (degrees < 0.0) degrees += 1.0;
    if (degrees > 1.0) degrees -= 1.0;
    if (kIsReversed) degrees = 1.0 - degrees;
    return (degrees * max + min);
  }

  public double getSimRaw() {
    return simState;
  }

  public double getSim() {
    double degrees = getSimRaw() - kOffset;
    if (degrees < 0.0) degrees += 1.0;
    if (degrees > 1.0) degrees -= 1.0;
    if (kIsReversed) degrees = 1.0 - degrees;
    return degrees * 360.0;
  }

  public double getSim(double min, double max) {
    double degrees = getSimRaw() - kOffset;
    if (degrees < 0.0) degrees += 1.0;
    if (degrees > 1.0) degrees -= 1.0;
    if (kIsReversed) degrees = 1.0 - degrees;
    return (degrees * max + min);
  }

  public void updateSim() {
    double out = (m_sourcePosition.get() * kGearRatio) % 1.0;
    if (kNoiseEnabled) out += kNoise.nextDouble() * (kNoise.nextBoolean() ? -1e-3 : 1e-3);

    if (Utils.isSimulation()) m_sim.set(out);
    simState = out;
  }
}
