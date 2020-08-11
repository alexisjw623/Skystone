package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Sam on 2/24/2020
 **/

@TeleOp(name="TeleOp: Mecanum_UPDATED", group="Linear Opmode")
public class TeleOp_MecanumTest_v1 extends LinearOpMode{
    Hardware_MecanumUPDATED pumpkin1 = new Hardware_MecanumUPDATED();
    double rotatePosition = 1, liftPosition, servoSpeed, rotateSpeed, fmoverPosition, armSpeed;
    double MIN_POSITION = 0; double MAX_POSITION = 1;
    double colorCondition;

    @Override
    public void runOpMode() {
        pumpkin1.init(hardwareMap);
        pumpkin1.clawControl.setPosition(1);
        waitForStart();

        servoSpeed = .25;
        rotateSpeed = .2;
        armSpeed = 0.3;

        while (opModeIsActive()) {

            //rotation - right joystick
            double rightX = -gamepad1.right_stick_x;
            //movement - left joystick
            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y;

            //driving
            pumpkin1.LFmotor.setPower(leftY + rightX + leftX );
            pumpkin1.RFmotor.setPower(leftY - rightX - leftX);
            pumpkin1.LBmotor.setPower(leftY + rightX - leftX);
            pumpkin1.RBmotor.setPower(leftY - rightX + leftX);


            /* COMPLIANT WHEELS - left trigger/right trigger */
            double wheelOuttake = gamepad1.right_trigger;
            double wheelIntake = -gamepad1.left_trigger;
            //
            if ( gamepad1.left_trigger > 0 ) {
                pumpkin1.LCompliantmotor.setPower(wheelIntake);
                pumpkin1.RCompliantmotor.setPower(wheelIntake);
            }
            else if ( gamepad1.right_trigger > 0){
                pumpkin1.LCompliantmotor.setPower(wheelOuttake);
                pumpkin1.RCompliantmotor.setPower(wheelOuttake);
            }
            else{
                pumpkin1.LCompliantmotor.setPower(0);
                pumpkin1.RCompliantmotor.setPower(0);
            }

            if ( gamepad1.right_stick_button ) {
                pumpkin1.rotateArmOut();
                pumpkin1.FourBarmotor.setPower(.3);
                sleep(pumpkin1.getMotorArmSleep());
                pumpkin1.FourBarmotor.setPower(0);
            }
            /* FOUR BAR - right/left bumper */
            boolean raiseBar = gamepad1.right_bumper;
            boolean lowerBar = gamepad1.left_bumper;

            if (raiseBar) pumpkin1.FourBarmotor.setPower(armSpeed);
            else if (lowerBar) pumpkin1.FourBarmotor.setPower(-armSpeed);
            else pumpkin1.FourBarmotor.setPower(0);

            /* Foundation movers - y and x */
            // put UP the block stealer servo using the X button
            if (gamepad1.x && fmoverPosition < MAX_POSITION) fmoverPosition= fmoverPosition + .5;
            // put DOWN the foundation mover servos using the Y button
            if (gamepad1.y && fmoverPosition > MIN_POSITION) fmoverPosition = fmoverPosition - .5;


            /* OPEN/CLOSE CLAW  - dpad up and dpad down */
            // close claw using DPAD_DOWN; 0 = open; 1 = closed
            if (gamepad1.dpad_down && rotatePosition < MAX_POSITION) rotatePosition += rotateSpeed;
            // up claw using DPAD_UP
            if (gamepad1.dpad_up && rotatePosition > MIN_POSITION) rotatePosition -= rotateSpeed;

            /* ARM IN/OUT - a and b*/
            // move arm up
            if (gamepad1.b && liftPosition < MAX_POSITION) liftPosition += servoSpeed;
            // move arm in
            if (gamepad1.a && liftPosition > .2) liftPosition -= servoSpeed;


            // set the servo values
            pumpkin1.clawControl.setPosition(Range.clip(rotatePosition, MIN_POSITION, MAX_POSITION));
            pumpkin1.liftClaw.setPosition(Range.clip(liftPosition, MIN_POSITION, MAX_POSITION));
            pumpkin1.fMover.setPosition(Range.clip(fmoverPosition, MIN_POSITION,MAX_POSITION));


            /*
             * TELEMETRY
             * sends info back to driver station using telemetry function
             */


            telemetry.addData("CONTROLS", "\nintake: LT   outtake: RT\narmup: RB  armdown: LB\nrotatein: dpad_l  rotateout: dp_r\n\n");
            //servo data

            telemetry.addData("rotatePosition", String.format("position=%.2f  actual=%.2f", rotatePosition, pumpkin1.clawControl.getPosition()));
            telemetry.addData("liftPosition", String.format("position=%.2f  actual=%.2f", liftPosition, pumpkin1.liftClaw.getPosition()));
            telemetry.addData("fmoverPosition", String.format("position=%.2f  actual=%.2f", fmoverPosition, pumpkin1.fMover.getPosition()));

            //color sensor data
            telemetry.addData("PARK COLOR SENSOR", "");
            telemetry.addData("Alpha", pumpkin1.parkColorS.alpha());
            telemetry.addData("Red  ", pumpkin1.parkColorS.red());
            telemetry.addData("Green", pumpkin1.parkColorS.green());
            telemetry.addData("Blue ", pumpkin1.parkColorS.blue());

            telemetry.addData("STONE COLOR SENSOR", "");
            telemetry.addData("Alpha", pumpkin1.stoneColorS.alpha());
            telemetry.addData("Red  ", pumpkin1.stoneColorS.red());
            telemetry.addData("Green", pumpkin1.stoneColorS.green());
            telemetry.addData("Blue ", pumpkin1.stoneColorS.blue());
            telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", pumpkin1.stoneDistance.getDistance(DistanceUnit.CM)));

            telemetry.addData("isSkystone", pumpkin1.isSkystone());
            telemetry.addData("isPressed", !pumpkin1.touchSensor.getState());

            telemetry.update();


        }
    }

}
