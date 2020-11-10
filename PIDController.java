package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PController extends LinearOpMode {

    DcMotor motor;
    ElapsedTime timer;

    double power;
    double target = 90;

    double kP = 0.2;
    double kI = 0.0001;
    double kD = 0.001

    double sum = 0;

    double pOutput = 0;
    double iOutput = 0;
    double dOutput = 0;

    double position = 0;
    double error = 0;

    double lastPosition, deltaR;
    double time, lastTime, deltaT;
    double velocity;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        while (opModeIsActive()) {

            //Proportional
            position = motor.getCurrentPosition();
            error = target - position;
            pOutput = kP * (error);

            //Integral
            sum = error * 20;
            iOutput = kI * sum;

            //Derivative
            deltaR = (position - lastPosition);
            time = timer.time();
            deltaT = (time - lastTime);
            velocity = (deltaR / deltaT);
            dOutput = kD * velocity;

            lastPosition = position;
            lastTime = time;

            
            power = pOutput + iOutput - dOutput;

            if(power > 0.5){
                power = 0.5;
            } else if (power < -0.5){
                power = -0.5;
            }

            telemetry.update();
            motor.setPower(power);


        }
        motor.setPower(0.0);
    }
}
