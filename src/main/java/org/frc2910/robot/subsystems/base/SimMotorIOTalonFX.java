package org.frc2910.robot.subsystems.base;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.frc2910.robot.util.ConditionalCallbackManager;

/**
 * SIMULATION implementation of MotorIO used by all BaseSubsystems that require TalonFX motor in simulation. Extends MotorIOTalonFX
 *
 * @see MotorIOTalonFX
 */
public class SimMotorIOTalonFX extends MotorIOTalonFX {
    private final DCMotorSim simMotor;

    private double lastUpdateTimestamp = 0.0;
    private final double forwardSoftwareLimit;
    private final double reverseSoftwareLimit;

    public SimMotorIOTalonFX(BaseMotorConfig config, ConditionalCallbackManager callbackManager) {
        super(config, callbackManager);

        // TODO why isn't gearing being automatically used by DCMotorSim
        this.simMotor = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        DCMotor.getKrakenX60Foc(1), config.getMomentOfInertia(), motorRotationsToMechanismUnitsRatio),
                DCMotor.getKrakenX60Foc(1),
                config.getPositionStandardDeviation(),
                config.getVelocityStandardDeviation());

        this.forwardSoftwareLimit = config.getMotorConfiguration().SoftwareLimitSwitch.ForwardSoftLimitThreshold;
        this.reverseSoftwareLimit = config.getMotorConfiguration().SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }

    // Needed because DC motor doesn't restrict based on the talon's limit configs. Conversions are private
    private double restrictMechanismPosition(double mechanismPosition) {
        return MathUtil.clamp(mechanismPosition, reverseSoftwareLimit, forwardSoftwareLimit);
    }

    private double getSimMechanismPosition() {
        return simMotor.getAngularPositionRotations() / motorRotationsToMechanismUnitsRatio;
    }

    @Override
    public void setMechanismPositionSetpoint(double mechanismPosition) {
        setMechanismPositionSetpoint(mechanismPosition, 0);
    }

    @Override
    public void setMechanismPositionSetpoint(double mechanismPosition, int slot) {
        simMotor.setAngle(
                ((restrictMechanismPosition(mechanismPosition) * motorRotationsToMechanismUnitsRatio) * 2 * Math.PI));
    }

    @Override
    public void setMechanismVelocityPerSecondSetpoint(double mechanismVelocityPerSecond) {
        setMechanismVelocityPerSecondSetpoint(mechanismVelocityPerSecond, 0);
    }

    @Override
    public void setMechanismVelocityPerSecondSetpoint(double mechanismVelocityPerSecond, int slot) {
        simMotor.setAngularVelocity((mechanismVelocityPerSecond * motorRotationsToMechanismUnitsRatio) * 2 * Math.PI);
    }

    @Override
    public void setOpenLoopDutyCycle(double dutyCycle) {
        double defaultMatchBatteryVoltage = 12.0;
        setVoltageOutput(defaultMatchBatteryVoltage * dutyCycle);
    }

    @Override
    public void setVoltageOutput(double voltage) {
        TalonFXConfiguration motorConfiguration = config.getMotorConfiguration();

        // Both SIM and Real stop the effects of voltage after going past soft limits
        if (motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable
                && voltage > 0
                && getSimMechanismPosition() >= forwardSoftwareLimit) {
            setMechanismPositionSetpoint(forwardSoftwareLimit);
            voltage = 0.0;
        } else if (motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable
                && voltage < 0
                && getSimMechanismPosition() <= reverseSoftwareLimit) {
            setMechanismPositionSetpoint(reverseSoftwareLimit);
            voltage = 0.0;
        }
        simMotor.setInputVoltage(addFriction(voltage, config.getFrictionVoltage()));
    }

    // TODO check to make sure voltage and friction being used accurately on simMotor
    private double addFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            return 0.0;
        } else {
            return (Math.abs(motorVoltage) - frictionVoltage) * Math.signum(motorVoltage);
        }
    }

    @Override
    public void updateInputs(MotorIOInputs inputs) {
        double timestamp = Timer.getFPGATimestamp(); // Time since roboRio on or code deploy
        simMotor.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        inputs.mechanismEndUnit = mechanismEndUnit;

        inputs.motorRawPositionRotations = simMotor.getAngularPositionRotations();
        inputs.motorVelocityRPS = simMotor.getAngularVelocityRPM() / 60.0;
        inputs.motorVoltage = simMotor.getInputVoltage();
        inputs.motorSupplyCurrentAmps = simMotor.getCurrentDrawAmps();
        inputs.mechanismRawPositionInMechanismUnits =
                inputs.motorRawPositionRotations / motorRotationsToMechanismUnitsRatio;
        inputs.mechanismVelocityPerSecondInMechanismUnits =
                inputs.motorVelocityRPS / motorRotationsToMechanismUnitsRatio;
    }
}
