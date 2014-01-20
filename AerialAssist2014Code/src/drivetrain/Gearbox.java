package drivetrain;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.templates.Utils;

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
        this(new Talon(frontChannel), new Talon(rearChannel));
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
        this(new Talon(frontChannel), new Talon(rearChannel), new Talon(midChannel));
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
     * @deprecated Returns the value inserted and not the real value
     * @return Value used in set
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

    public void set(double speed) {
        speed = Utils.limitSpeed(speed);
        frontController.set(speed);
        rearController.set(speed);
        if (hasThreeControllers()) {
            midController.set(speed);
        }
    }

    public void disable() {
        frontController.disable();
        rearController.disable();
        if (hasThreeControllers()) {
            midController.disable();
        }
    }

    public void pidWrite(double speed) {
        set(speed);
    }

}
