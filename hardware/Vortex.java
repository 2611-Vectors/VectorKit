package frc.robot.VectorKit.hardware;

import static com.revrobotics.PersistMode.kNoPersistParameters;
import static com.revrobotics.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;

public class Vortex extends SubsystemBase {
    private final SparkClosedLoopController pidController;
    private final SparkBaseConfig config;

    private final SparkFlex m_motor;

    private PidTuner tuner = null;

    public Vortex(int ID) {
        m_motor = new SparkFlex(ID, kBrushless);
        pidController = m_motor.getClosedLoopController();

        config = new SparkFlexConfig();

        m_motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void addTuner(PidTuner tuner) {
        this.tuner = tuner;
    }

    public void setBrakeMode(boolean brakeMode) {
        config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
    }

    public Command set(Supplier<Double> speed) {
        return run(() -> m_motor.set(speed.get()));
    }

    public void addFollower(Vortex follower, MotorAlignmentValue motorAlignment) {
        follower.config.follow(m_motor);
        follower.m_motor.configure(follower.config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public Command setVelocity(Supplier<Double> vel, Supplier<AngularVelocityUnit> unit) {
        return run(() -> pidController.setSetpoint(RPM.convertFrom(vel.get(), unit.get()), kVelocity));
    }

    public void setInverted(InvertedValue direction) {
        config.inverted(direction == InvertedValue.Clockwise_Positive);
        m_motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void setPID(double kP, double kI, double kD) {
        config.closedLoop.pid(kP, kI, kD);
        m_motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void setFF(double kS, double kV) {
        config.closedLoop.feedForward.sv(kS, kV);
        m_motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void updateFromTuner(PidTuner tuner) {
        config.closedLoop.pid(tuner.getP(), tuner.getI(), tuner.getD());
        config.closedLoop.feedForward.sv(tuner.getS(), tuner.getV());
        m_motor.configure(config, kNoResetSafeParameters, kNoPersistParameters);
    }

    @Override
    public void periodic() {
        if (tuner != null) if (tuner.updated()) updateFromTuner(tuner);
    }

    /**
     * The relationship between two motors in a mechanism. Depending on hardware setup, one motor may
     * be inverted relative to the other motor.
     */
    public enum MotorAlignmentValue {
        /**
         * The two motor directions are aligned. Positive output on both motors moves the mechanism
         * forward/backward.
         */
        Aligned(),
        /**
         * The two motor directions are opposed. To move forward/backward, one motor needs positive
         * output, and the other needs negative output.
         */
        Opposed(),
        ;
    }
}
