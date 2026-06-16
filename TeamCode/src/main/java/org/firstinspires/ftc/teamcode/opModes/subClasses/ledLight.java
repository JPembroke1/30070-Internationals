package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Configurable
@TeleOp
public class ledLight {
    public Servo led;

    led = harwaremap.get(Servo.class, "led");

    public static double red = 0.3;
    public static double green = 0.5;



}
