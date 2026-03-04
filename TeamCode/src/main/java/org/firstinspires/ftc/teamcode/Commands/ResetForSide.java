package org.firstinspires.ftc.teamcode.Commands;

import com.pedropathing.follower.Follower;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.PosePersistency;

public class ResetForSide extends InstantCommand {
    public ResetForSide(Follower follower) {
        super(() -> PosePersistency.applyReset(follower));
    }

}
