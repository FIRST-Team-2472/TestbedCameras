package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    static class FieldConstants {
        static final double length = Units.feetToMeters(54);
        static final double width = Units.feetToMeters(27);
}
static class SensorConstants {
    static final Transform3d robotToCam =
            new Transform3d(new Translation3d(0, 0.0, 0),new Rotation3d(0, 0,0));
             // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    static final String camera1 = "Camera1";
    static final String camera2 = "Camera2";
    static final String camera3 = "Camera3";
    static final String camera4 = "Camera4";
    
}
}
