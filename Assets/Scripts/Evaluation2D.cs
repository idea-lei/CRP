using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Evaluation2D : Singleton<Evaluation2D> {
    public int TotalContainerOut = 0;
    public int TotalRelocation = 0;

    public void UpdateValue(int o, int r) {
        TotalContainerOut += o;
        TotalRelocation += r;
    }

    private void OnDestroy() {
        Debug.Log($"TotalContainerOut {TotalContainerOut}, TotalRelocation {TotalRelocation}, relocation ratio {TotalRelocation / (float)TotalContainerOut}");
    }
}