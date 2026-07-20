/// <summary>
/// Selects which training curriculum this scene/agent uses.
/// </summary>
public enum RobotTrainingMode
{
    /// <summary>Arm locked to the floor-pickup pose; agent drives and pans camera only.</summary>
    FixedArm = 0,

    /// <summary>Agent controls shoulder, elbow and wrist roll in addition to base motion.</summary>
    MobileArm = 1
}
