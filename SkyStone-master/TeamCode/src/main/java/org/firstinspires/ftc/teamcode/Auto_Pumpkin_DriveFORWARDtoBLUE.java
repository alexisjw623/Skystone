package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Created by Sam on 2/27/2020
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="SENS: DriveFORWARDtoBLUE", group="Pumpkin: BLUE")
public class Auto_Pumpkin_DriveFORWARDtoBLUE extends LinearOpMode{
    Hardware_MecanumUPDATED autopumpkin = new Hardware_MecanumUPDATED();
    public void runOpMode(){

        autopumpkin.init(hardwareMap);

        waitForStart();

        //moves FORWARD until it sees blue
        while (!autopumpkin.parkBlue())
        {
            movement(.3,.3,.3,.3);
        }

        movement(0,0,0,0);
    }

    public void movement(double LF, double LB, double RF, double RB)
    {
        autopumpkin.LFmotor.setPower(LF);
        autopumpkin.LBmotor.setPower(LB);
        autopumpkin.RFmotor.setPower(RF);
        autopumpkin.RBmotor.setPower(RB);
    }
}
