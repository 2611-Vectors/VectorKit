package frc.robot.VectorKit.hardware;

import static com.revrobotics.PersistMode.kNoPersistParameters;
import static com.revrobotics.ResetMode.kNoResetSafeParameters;
import static com.revrobotics.spark.SparkBase.ControlType.kVelocity;
import static com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VectorKit.tuners.PidTuner;
import java.util.function.Supplier;

public class Neo extends SubsystemBase {
    private final SparkMax m_Motor;

    private final SparkClosedLoopController m_PidController;
    private final SparkMaxConfig m_Config;

    private PidTuner tuner = null;

    public Neo(int ID) {
        m_Motor = new SparkMax(ID, kBrushless);
        m_PidController = m_Motor.getClosedLoopController();

        m_Config = new SparkMaxConfig();

        m_Config.closedLoop
                .outputRange(-1, 1)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.0001)
                .i(0)
                .d(0)
                .feedForward
                .kS(0.0)
                .kV(0.0);

        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void addTuner(PidTuner tuner) {
        this.tuner = tuner;
    }

    public void setBrakeMode(boolean brakeMode) {
        m_Config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void setOutputRange(double min, double max) {
        m_Config.closedLoop.outputRange(min, max);
    }

    public Command set(Supplier<Double> speed) {
        return run(() -> m_Motor.set(speed.get()));
    }

    public void addFollower(Neo follower, MotorAlignmentValue motorAlignment) {
        follower.m_Config.follow(m_Motor, motorAlignment == MotorAlignmentValue.Opposed);
        follower.m_Motor.configure(follower.m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public Command setVelocity(Supplier<Double> vel, Supplier<AngularVelocityUnit> unit) {
        return run(() -> m_PidController.setSetpoint(RPM.convertFrom(vel.get(), unit.get()), kVelocity));
    }

    public double getRPM() {
        return m_Motor.getEncoder().getVelocity();
    }

    public double getAmperage() {
        return m_Motor.getOutputCurrent();
    }

    public void stop() {
        m_Motor.set(0.0);
    }

    public void setInverted(InvertedValue direction) {
        m_Config.inverted(direction == InvertedValue.Clockwise_Positive);
        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void setPID(double kP, double kI, double kD) {
        m_Config.closedLoop.pid(kP, kI, kD);
        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void setFF(double kS, double kV) {
        m_Config.closedLoop.feedForward.sv(kS, kV);
        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
    }

    public void updateFromTuner(PidTuner tuner) {
        m_Config.closedLoop.pid(tuner.getP(), tuner.getI(), tuner.getD());
        m_Config.closedLoop.feedForward.sv(tuner.getS(), tuner.getV());
        m_Motor.configure(m_Config, kNoResetSafeParameters, kNoPersistParameters);
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
