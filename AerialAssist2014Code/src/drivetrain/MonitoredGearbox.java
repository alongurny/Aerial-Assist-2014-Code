package drivetrain;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.templates.Vars;

/**
 *
 * @author alon & roni && ETHAN RULES!!
 */
public class MonitoredGearbox extends Gearbox {

    private double wheelRadius;
    private Encoder encoder;
    private double scaleSpeedFactor = 1.0;
    
    public MonitoredGearbox(int aEncoder, int bEncoder, double wheelRadius, int frontChannel, int rearChannel) {
        this(new Encoder(aEncoder,bEncoder), wheelRadius, frontChannel, rearChannel);
    }
    
    public MonitoredGearbox(int aEncoder, int bEncoder, double wheelRadius, int frontChannel, int rearChannel, int midChannel) {
        this(new Encoder(aEncoder,bEncoder), wheelRadius, frontChannel, rearChannel, midChannel);
    }
    
    public MonitoredGearbox(Encoder encoder, double wheelRadius, int frontChannel, int rearChannel) {
        this(encoder, wheelRadius, new Jaguar(frontChannel), new Jaguar(rearChannel));
    }

    public MonitoredGearbox(Encoder encoder, double wheelRadius, SpeedController front, SpeedController rear) {
        this(encoder, wheelRadius, front, rear, null);
    }

    public MonitoredGearbox(Encoder encoder, double wheelRadius, int frontChannel, int rearChannel, int midChannel) {
        this(encoder, wheelRadius, new Jaguar(frontChannel), new Jaguar(rearChannel), new Jaguar(midChannel));
    }

    public MonitoredGearbox(Encoder encoder, double wheelRadius, SpeedController front, SpeedController rear, SpeedController mid) {
        super(front, rear, mid);
        setEncoder(encoder);
        setWheelRadius(wheelRadius);
    }

    /**
     * Get the distance the robot has driven since the last reset.
     * @return The distance the robot has driven
     */
    public double getDistance() {
        return encoder.getDistance();
    }
    /**
     * Get the angle of the encoder.
     * @return Angle
     */
    public double getAngle() {
        return encoder.get() * 2 * Math.PI;
    }
    /**
     * Get the velocity of the robot.
     * @return Velocity of the robot
     */
    public double getVelocity() {
        return encoder.getRate();
    }
    
    /**
     * Get the angular velocity of the encoder.
     * @return Angular velocity of the encoder
     */
    public double getAngularVelocity() {
        return getVelocity() / wheelRadius;
    }

    /**
     * Reset the distance the robot has driven to 0.
     */
    public void reset() {
        encoder.reset();
    }
    /**
     * Starts the encoder.
     */
    public void startEncoders() {
        encoder.start();
    }
    /**
     * Stops the encoder.
     */
    public void stopEncoders() {
        encoder.stop();
    }
    /**
     * Set a new Encoder.
     * @param encoder New encoder
     */
    private void setEncoder(Encoder encoder) {
        this.encoder = encoder;
    }
    /**
     * 
     * Set a speed to the Gearbox.
     * @param speed Speed to set to the Gearbox.
     */
    public void set(double speed) {
        speed = scaleSpeedFactor * speed;
        super.set(speed);
    }

    /**
     * 
     * @param radius The radius of the wheels in centimeters
     */
    public void setWheelRadius(double radius) {
        this.wheelRadius = radius;
        encoder.setDistancePerPulse(2 * Math.PI * radius / Vars.Drivetrain.ENCODER_TICKS);
    }

    /**
     * Get the radius of the wheels.
     * @return The radius of the wheels in centimeters
     */
    public double getWheelRadius() {
        return wheelRadius;
    }

    /**
     * Get the encoder used in this Gearbox.
     * @return Encoder
     */
    public Encoder getEncoder() {
        return encoder;
    }
    /**
     * Set a new speed factor to the Gearbox.
     * This function is used in scaling the gearbox speed.
     * The wanted speed is multiplied by this factor to get the actual speed.
     * 
     * @param factor 
     */
    public void setSpeedFactor(double factor) {
        this.scaleSpeedFactor = factor;
    }
}
