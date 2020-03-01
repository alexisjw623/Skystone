package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 2/9/20
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NO_SENSORS: FORWARD AND RIGHT", group="Pumpkin: NoSensors")
public class Auto_Pumpkin_NOSENSOR_ForwardRIGHT extends LinearOpMode{
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        //move FORWARD
        autopumpkin.movement(.75,.75,.75,.75);
        sleep (autopumpkin.getForwardAutoSleep());
        autopumpkin.movement(.5,-.5,-.5,.5);
        sleep(autopumpkin.getDistanceToParkSleep());

        autopumpkin.stopAllMotors();
    }
}
