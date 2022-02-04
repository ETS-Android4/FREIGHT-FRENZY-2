package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servo_brat {
    public Servo servo = null;

    public static double POS_JOS = 0.20;
    public static double POS_1st = 0.7;
    public static double POS_2nd = 0.91;
    public static double POS_SUS = 0.98;

    public servo_brat(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "servoBrat");
        jos();
    }

    public void setServoPositions(double pos1) {
        if(pos1 > -1.0) {
            servo.setPosition(pos1);
        }
    }

    public void jos() { setServoPositions(POS_JOS);}
    public void first() { setServoPositions(POS_1st);}
    public void second() { setServoPositions(POS_2nd);}
    public void sus() { setServoPositions(POS_SUS);}

}