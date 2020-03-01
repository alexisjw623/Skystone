package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 12/10/19
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Auto_Pumpkin: Forward//Drive LEFT to Blue", group="Pumpkin: BLUE")
public class Auto_Pumpkin_Forward_DriveLEFTtoBLUE extends LinearOpMode{
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        //move FORWARD
        autopumpkin.movement(.75,.75,.75,.75);
        sleep (autopumpkin.getForwardAutoSleep());

        //moves LEFT until it sees blue
        while (!autopumpkin.parkBlue())
        {
            autopumpkin.movement(-.5,.5,.5,-.5);
        }

        autopumpkin.stopAllMotors();
    }
}
