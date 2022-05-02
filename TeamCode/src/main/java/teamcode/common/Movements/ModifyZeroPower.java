package teamcode.common.Movements;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ModifyZeroPower extends Movement{

    private DcMotor.ZeroPowerBehavior behavior;

    public ModifyZeroPower(DcMotor.ZeroPowerBehavior behavior){
        this.behavior = behavior;
    }

    public DcMotor.ZeroPowerBehavior getBehavior() {
        return behavior;
    }
}
