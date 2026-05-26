package org.firstinspires.ftc.teamcode.writtenCode.controllers;


import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.writtenCode.RobotMap;

@Config

@Configurable
public class HoodController {
    public enum HoodStatus{
        INIT,
        RUNTO;
    }
    public HoodStatus currentStatus= HoodStatus.INIT;
    public HoodStatus previousStatus=null;

    public static double hoodInitPosition=1;




    public Servo hood1 = null;
    public Servo hood2 = null;

    public HoodController(RobotMap robot) {
        this.hood1 = robot.hood1;
        this.hood2 = robot.hood2;
    }
    public void update(double runto_target){

        if(currentStatus!=previousStatus || currentStatus== HoodStatus.RUNTO)
        {
            previousStatus=currentStatus;
            switch(currentStatus)
            {
                case INIT:
                {
                    this.hood1.setPosition(hoodInitPosition);
                    this.hood2.setPosition(hoodInitPosition);
                    break;
                }
                case RUNTO:
                {
                    this.hood1.setPosition(runto_target);
                    this.hood2.setPosition(runto_target);
                    break;
                }

            }
        }
    }

}

