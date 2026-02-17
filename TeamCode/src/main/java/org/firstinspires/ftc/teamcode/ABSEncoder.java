package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Configurable
public class ABSEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;
    private final AnalogInput encoder;
    private double offset, analogRange;
    private boolean inverted;

    private double normalizeAngle(double angle) {
        double twoPi = 2.0 * Math.PI;
        angle = angle % twoPi;
        if (angle < 0) {
            angle += twoPi;
        }
        return angle;
    }

    private double normalizeDelta(double angle) {
        double twoPi = 2.0 * Math.PI;
        angle = angle % twoPi;
        if (angle > Math.PI) {
            angle -= twoPi;
        } else if (angle < -Math.PI) {
            angle += twoPi;
        }
        return angle;
    }

    /**
     * Make a regular encoder with the default range of DEFAULT_RANGE
     */
    public ABSEncoder(AnalogInput enc){
        this(enc, DEFAULT_RANGE);
    }

    /**
     * Make a regular encoder with a custom range
     *
     * @param enc the encoder
     * @param aRange the range of the encoder
     */
    public ABSEncoder(AnalogInput enc, double aRange){
        encoder = enc;
        analogRange = aRange;
        offset = 0;
        inverted = false;
    }

    /**
     * Sets the zero of the encoder
     *
     * @param off voltage to set the zero to
     *
     * @return this
     */
    public ABSEncoder zero(double off){
        offset = off;
        return this;
    }
    /**
     * Sets the encoder to be inverted
     *
     * @param invert true to invert the encoder
     */
    public ABSEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }

    /**
     * Gets the direction (if its inverted)
     *
     * @returns true if the encoder is inverted
     */
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;

    /**
     * Gets the current position with angle normalization
     *
     * @returns the current position of the encoder in radians
     */
    public double getCurrentPosition() {
        double pos = normalizeAngle((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * Math.PI*2 - offset);
        //checks for crazy values when the encoder is close to zero
        if(!VALUE_REJECTION || Math.abs(normalizeDelta(pastPosition)) > 0.1 || Math.abs(normalizeDelta(pos)) < 1) pastPosition = pos;

        return pastPosition;
//        return MathUtils.round(pastPosition, 3);
    }

    /**
     * Returns this encoder as an AnalogInput
     *
     * @return analoginput of the encoder
     */
    public AnalogInput getEncoder() {
        return encoder;
    }

    /**
     * Returns the current voltage of the encoder
     *
     * @return voltage of the encoder
     */
    public double getVoltage(){
        return encoder.getVoltage();
    }
}
