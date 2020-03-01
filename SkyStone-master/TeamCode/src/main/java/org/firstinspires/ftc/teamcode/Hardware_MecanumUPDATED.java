package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 *  Created by Sam on 2/26/2020
 ****************************
 *  VERTICALL HUB: HUB 2
 *  LC 0         RC 1
 *        HUB 2
 *  FB 2
 *
 *  HUB 2: address 1
 *  Digital Device 1: touchS
 ****************************
 *  HORIZONTAL HUB: HUB 1
 *  LF 0         RF 1
 *        HUB 1
 *  LB 2         RB 3
 *
 *  HUB 1: address 2
 *  Servo Port 4: clawControl
 *  Servo Port 5: liftClaw
 *  I2C Bus 0: parkColorS
 *  I2C Bus 1: stoneColorS/stoneDistance
 ****************************
 */

class Hardware_MecanumUPDATED {

    public enum Direction
    {
        FORWARD, BACKWARD, STRAFE_RIGHT, STRAFE_LEFT, DIAGONAL_LEFT, DIAGONAL_RIGHT;
    }

    //Drive train:
    DcMotor LFmotor;
    DcMotor LBmotor;
    DcMotor RFmotor;
    DcMotor RBmotor;

    DcMotor LCompliantmotor;
    DcMotor RCompliantmotor;

    DcMotor FourBarmotor;

    Servo clawControl;
    Servo liftClaw;
    Servo fMover;

    ColorSensor parkColorS;
    ColorSensor stoneColorS;
    DistanceSensor stoneDistance;
    //TouchSensor touchSensor;
    DigitalChannel touchSensor;

    //sleep calls for various autonomous modes
    private long motorArmSleep = 1200;
    private long forwardAutoSleep = 1000;
    private long distanceToParkSleep = 1500;

    private int colorThresh = 30;

    public void init(HardwareMap hwMap){

        // assigns names
        LFmotor = hwMap.dcMotor.get("LFmotor");
        LBmotor = hwMap.dcMotor.get("LBmotor");
        RFmotor = hwMap.dcMotor.get("RFmotor");
        RBmotor = hwMap.dcMotor.get("RBmotor");

        LCompliantmotor = hwMap.dcMotor.get("RCmotor");
        RCompliantmotor = hwMap.dcMotor.get("LCmotor");

        FourBarmotor = hwMap.dcMotor.get("4Bmotor");

        clawControl = hwMap.servo.get("clawControl");
        liftClaw = hwMap.servo.get("liftClaw");
        fMover = hwMap.servo.get("fMover");
        parkColorS = hwMap.get(ColorSensor.class, "parkColorS");
        stoneColorS = hwMap.get(ColorSensor.class, "stoneColorS");
        stoneDistance = hwMap.get(DistanceSensor.class, "stoneColorS");
        //touchSensor = hwMap.get(TouchSensor.class, "touchS");

        // get a reference to our digitalTouch object.
        touchSensor = hwMap.get(DigitalChannel.class, "touchS");
        // set the digital channel to input.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

        // set brakes on motors
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LCompliantmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RCompliantmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FourBarmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set direction of motors facing opposite directions
        // DcMotors: Clockwise by default; clockwise on left side = forward
        // FRONT = non-compliant wheels
        LFmotor.setDirection(DcMotor.Direction.FORWARD);
        LBmotor.setDirection(DcMotor.Direction.FORWARD);
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        LCompliantmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RCompliantmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FourBarmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        stopAllMotors();
        closeClaw();
        rotateArmIn();
    }

    public void stopAllMotors(){
        //set pwr to 0
        LFmotor.setPower(0.0);
        LBmotor.setPower(0.0);
        RFmotor.setPower(0.0);
        RBmotor.setPower(0.0);

        LCompliantmotor.setPower(0.0);
        RCompliantmotor.setPower(0.0);
        FourBarmotor.setPower(0);
    }

    public void moveMecanum( int leftY, int rightX, int leftX){
        double maxPower = 0.5;
        double minPower = -0.5;

        double LFpower = Range.clip(leftY + rightX + leftX, minPower, maxPower);
        double RFpower = Range.clip(leftY - rightX - leftX, minPower, maxPower);
        double LBpower = Range.clip(leftY + rightX - leftX, minPower, maxPower);
        double RBpower = Range.clip(leftY - rightX + leftX, minPower, maxPower);

        LFmotor.setPower(LFpower);
        RFmotor.setPower(RFpower);
        LBmotor.setPower(LBpower);
        RBmotor.setPower(RBpower);
    }

    public void movement(double LF, double LB, double RF, double RB)
    {
        LFmotor.setPower(LF);
        LBmotor.setPower(LB);
        RFmotor.setPower(RF);
        RBmotor.setPower(RB);
    }

    public void openClaw(){
        clawControl.setPosition(0);
    }
    public void openClawSlight() { clawControl.setPosition(.4);}

    public void closeClaw(){
        clawControl.setPosition(1);
    }

    public void rotateArmIn(){
        liftClaw.setPosition(0);
    }

    public void rotateArmOut(){
        liftClaw.setPosition(0.98);
    }

    public long getMotorArmSleep(){
        return motorArmSleep;
    }
    public long getForwardAutoSleep(){
        return forwardAutoSleep;
    }
    public long getDistanceToParkSleep(){
        return distanceToParkSleep;
    }

    public boolean isSkystone() {
        float hsvValues[] = {0F, 0F, 0F};

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        Color.RGBToHSV((int) (stoneColorS.red() * SCALE_FACTOR),
                (int) (stoneColorS.green() * SCALE_FACTOR),
                (int) (stoneColorS.blue() * SCALE_FACTOR),
                hsvValues);


        //BLACK and WHITE
        if (hsvValues[0] > 100 && hsvValues[0] < 150) return true;
        return false;

    }

    public boolean parkBlue(){
        //return true if it sees blue
        return parkColorS.blue() > parkColorS.red() && parkColorS.blue() - parkColorS.red() > colorThresh;
    }

    public boolean parkRed(){
        //return true if it sees red
        return parkColorS.red() > parkColorS.blue() && parkColorS.red() - parkColorS.blue() > colorThresh;
    }
}