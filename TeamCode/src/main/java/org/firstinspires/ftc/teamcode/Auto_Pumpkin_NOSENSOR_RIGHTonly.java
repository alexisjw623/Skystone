package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 2/24/2020
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NO_SENSORS:  Drive RIGHT ONLY", group="Pumpkin: NoSensors")
public class Auto_Pumpkin_NOSENSOR_RIGHTonly extends LinearOpMode{
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();
        autopumpkin.movement(.5,-.5,-.5,.5);
        sleep(autopumpkin.getDistanceToParkSleep());
        autopumpkin.stopAllMotors();

    }
}
