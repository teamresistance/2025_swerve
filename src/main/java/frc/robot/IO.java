
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Button;

public class IO {
    public static Joystick joystickOne = new Joystick(0);
    public static JoystickButton btn1 = new JoystickButton(joystickOne, 1);

    public IO() {
        btn1.whenPressed(/*Something here */);
    }
}
