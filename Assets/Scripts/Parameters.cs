public static class Parameters {

    #region Container
    public static readonly float ContainerHeight = 2.5f;
    public static readonly float ContainerWidth = 2.5f;
    public static readonly float ContainerLength_Long = 12f;
    public static readonly float ContainerLength_Short = 6f;
    public static readonly int MaxContainerWeight = 20;
    #endregion

    #region PlayGround
    public static int DimZ = 3;
    public static int DimX = 1;
    public static int MaxLayer = 4;
    
    public static readonly int MinDim = 0; // this is only for test (random generation), should be smaller than the min Dim value

    public static readonly float Gap_Container = 1f;
    public static readonly float Gap_Field = 5f;
    public static readonly float SpawnInterval = 10f;

    public static readonly float PossibilityOfNewOutField = 0.001f; // this is only for test (random generation)
    #endregion

    #region Movement
    public static readonly float Vy_Loaded = 10f; // m/min
    public static readonly float Vy_Unloaded = 30f;
    public static readonly float Vx_Loaded = 15f;
    public static readonly float Vx_Unloaded = 20f;
    public static readonly float Vz_Loaded = 20f;
    public static readonly float Vz_Unloaded = 25f;

    public static readonly float T_adjust = 10f; // /s

    public static readonly float Ez = 0.001f;   // energy cost per meter
    public static readonly float Ex = 0.1f;
    #endregion

    #region Errors
    // this value should be larger than v_magnitude * deltatime
    public static readonly float DistanceError = 0.3f;
    public static readonly float SqrDistanceError = 0.548f; // use this to avoid the sqrt calculation
    #endregion

    #region Simulation
    public static readonly float EventDelay = 1f;
    public static readonly int MaxInFieldsWaiting = 7; // this is only for test (random generation)
    public static readonly int MaxInFieldsSpawn = 10000; // can be regarded as the simulation duration
    public static float InFieldGenerationInterval => DimX * 4f; // this is only for test (random generation)

    public static readonly float PossibilityOfDelay = 0.1f;
    public static float SetDelayInterval => DimZ * DimX * MaxLayer * 5f;
    public static int TrainingDim = 7;
    public static float TimeScale = 50;
    #endregion
}
