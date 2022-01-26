package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class TurnException extends RuntimeException{
    private static final long serialVersionUID = 1L;

    public TurnException(String msg){
        super(msg);
    }
}
