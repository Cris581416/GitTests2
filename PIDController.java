package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDController extends LinearOpMode {

    //Setup
    DcMotor motor;

    //Derivative
    ElapsedTime timer;

    //Setup
    double power;
    double target = 90;

    //Proportional
    double kP = 0.2;
    double pOutput = 0.0;
    double position = 0.0;
    double error = 0.0;

    //Integral
    double kI = 0.0001;
    double sum = 0.0;
    double iOutput = 0.0;

    //Derivative
    double kD = 0.001;
    double dOutput = 0.0;
    double lastPosition, deltaR;
    double time, lastTime, deltaT;
    double velocity;


    @Override
    public void runOpMode() throws InterruptedException {
        //Setup
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Derivative
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        //Setup
        while(opModeIsActive()){
            //Proportional
            position = motor.getCurrentPosition();
            error = target - position;
            pOutput = kP * (error);
            motor.setPower(power);

            //Integral
            sum = (error * 20);
            iOutput = (kI * sum);

            //Derivative
            deltaR = (position - lastPosition);
            time = timer.time();
            deltaT = (time - lastTime);
            velocity = (deltaR / deltaT);
            dOutput = (kD * velocity);

            //Update for the Next Loop
            lastPosition = position;
            time = lastTime;

            //Power
            power = pOutput + iOutput - dOutput;

        }
        motor.setPower(0.0);
    }
}