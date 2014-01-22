package drivetrain;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;

/**
 *
 * @author asaf
 */
public class Gearbox implements SpeedController {

    private SpeedController frontController, rearController, midController;

    /**
     * Create new Gearbox with Jaguars in the written ports.
     *
     * @param frontChannel channel of the first SpeedController
     * @param rearChannel channel of the second SpeedController
     */
    public Gearbox(int frontChannel, int rearChannel) {
        this(new Jaguar(frontChannel), new Jaguar(rearChannel));
    }

    /**
     * Create a Gearbox with the speedControllers;
     *
     * @param front front SpeedController
     * @param rear rear SpeedController
     */
    public Gearbox(SpeedController front, SpeedController rear) {
        this(front, rear, null);
    }

    public Gearbox(int frontChannel, int rearChannel, int midChannel) {
        this(new Jaguar(frontChannel), new Jaguar(rearChannel), new Jaguar(midChannel));
    }

    /**
     * Create a Gearbox with the speedControllers;
     *
     * @param front front SpeedController
     * @param rear rear SpeedController
     * @param mid middle SpeedController
     */
    public Gearbox(SpeedController front, SpeedController rear, SpeedController mid) {
        this.frontController = front;
        this.rearController = rear;
        this.midController = mid;
    }

    public boolean hasThreeControllers() {
        return midController != null;
    }

    public void stop() {
        set(0);
    }

    /**
     * @deprecated Does not return the actual speed of the Gearbox
     * @return Speed of the Gearbox
     */
    public double get() {
        return frontController.get();
    }

    /**
     * @deprecated Byte primitive type is not used
     * @param speed
     * @param b
     */
    public void set(double speed, byte b) {
        set(speed);
    }

    /**
     * 
     * @param speed Speed of the Gearbox
     */
    public void set(double speed) {
        if (speed > 1) {
            speed = 1;
        } else if (speed < -1) {
            speed = -1;
        }
        frontController.set(speed);
        rearController.set(speed);
        if (hasThreeControllers()) {
            midController.set(speed);
        }
    }

    /**
     * Disable.
     */
    public void disable() {
        frontController.disable();
        rearController.disable();
        if (hasThreeControllers()) {
            midController.disable();
        }
    }

    /**
     * Same as set(speed).
     * Use in PID computations.
     * @param speed 
     */
    public void pidWrite(double speed) {
        set(speed);
    }
}
