package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class color_sensor {
    public ColorSensor color;

    public color_sensor(HardwareMap hwMap) {
        color = hwMap.get(ColorSensor.class, "color");
    }
}