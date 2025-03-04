package frc.utils;

public class GampiecesFsm {
    public static enum Gamepieces{
        ALGAE,
        CORAL,
        BOTH,
        NONE
    }
    public static Gamepieces gamepieceInRobot=Gamepieces.CORAL;
    public static Gamepieces activeGamepiece=Gamepieces.CORAL;
}
