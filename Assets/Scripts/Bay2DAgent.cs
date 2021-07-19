using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public static class ToolFunctions {
    public static void RemoveAtIndices<T>(ref List<T> list, int[] a) {
        if (a is null || a.Length == 0) return;
        var sorted = a.ToList();
        sorted.Sort();
        var stack = new Stack<int>();
        foreach (var i in sorted) {
            stack.Push(i);
        }
        while (stack.Count > 0) {
            list.RemoveAt(stack.Pop());
        }
    }
}

public class Bay2DAgent : Agent {
    int count = 0;

    Bay bay;
    int maxLabel = (Parameters.DimZ - 1) * Parameters.MaxLayer + 1;
    readonly float blockingDegreeCoefficient = 100;
    int blockingDegreeOfState;
    int relocationTimes = 0;

    LastOperation lastOperation = new LastOperation();

    EnvironmentParameters envParams;
    BufferSensorComponent bs;

    public override void Initialize() {
        envParams = Academy.Instance.EnvironmentParameters;
        bs = GetComponent<BufferSensorComponent>();
        Debug.Assert(bs != null);
    }

    public override void OnEpisodeBegin() {
        relocationTimes = 0;
        lastOperation = new LastOperation();

        //int lowerRange = (int)envParams.GetWithDefault("amount", 16);
        //maxLabel = UnityEngine.Random.Range(lowerRange, 16 + 1);
        //maxLabel = 26;

        if (!Academy.Instance.IsCommunicatorOn) {
            if (count++ >= 100) {
                name = "finished";
                return;
            }
        }
        bay = new Bay(Parameters.DimZ, Parameters.MaxLayer, maxLabel);
        //Debug.Log(bay);
        Invoke(nameof(nextOperation), 0.1f); // to make sure all the commands are finished in this func
    }

    public override void CollectObservations(VectorSensor sensor) {

        var bd = bay.BlockingDegrees;
        //blockingDegreeOfState = bd.Sum();

        var layout = bay.LayoutAs2DArray;

        var ob = new List<List<float>>();
        for (int z = 0; z < bay.DimZ; z++) {
            // list is index obervation, it should contain containers information
            var list = new List<float>();

            // one hot, represent the z-index -- dimZ
            var oh = new float[bay.DimZ];
            oh[z] = 1;
            list.AddRange(oh);

            // dimZ -- 1
            list.Add(z / (float)bay.DimZ);

            // can pickup -- 1
            // 1. can not be empty 2. last time success or (unsuccess but lastOperation.z0 != z)
            bool canPickup =
                !bay.IndexEmpty(z) && (lastOperation.success || (!lastOperation.success && lastOperation.z0 != z));
            list.Add(canPickup ? 1 : 0);

            // can stack -- 1
            bool canStack =
                !bay.IndexFull(z) && (lastOperation.success || (!lastOperation.success && lastOperation.z1 != z));
            list.Add(canStack ? 1 : 0);

            // blockingDegree of stack -- 1
            // since the value is always negative, use mirror value to restrict ob range in [0,1]
            list.Add(-bd[z] / blockingDegreeCoefficient);

            // container info -- maxTier * (2 + maxTier)
            for (int t = 0; t < bay.MaxTier; t++) {

                // oh -- maxTier
                var ohStack = new float[bay.MaxTier];
                ohStack[t] = 1;
                list.AddRange(ohStack);

                // is moveable -- 1
                list.Add(t == 0 && !(layout[z, t] is null) ? 1 : 0);

                // priority, the larger, the later out -- 1
                list.Add(layout[z, t] is null ? 0 : (1 - layout[z, t].priority / (float)bay.MaxLabel));
            }

            Debug.Assert(list.Count == bay.DimZ + 4 + bay.MaxTier * (2 + bay.MaxTier));
            Debug.Assert(list.All(l => l <= 1 && l >= 0));
            ob.Add(list);
        }

        // remove the avoided index
        //ToolFunctions.RemoveAtIndices(ref ob, indicesToAvoid);

        var rnd = new System.Random();
        ob.OrderBy(n => rnd.Next());

        foreach (var l in ob) {
            bs.AppendObservation(l.ToArray());
        }

        //var observation = new List<float>();
        //foreach (var o in ob) {
        //    observation.AddRange(o);
        //}
        //observation.AddRange(new float[bay.DimZ * (bay.DimZ + bay.MaxTier + 3) - observation.Count]);

        //sensor.AddObservation(observation);
    }

    public override void Heuristic(in ActionBuffers actionsOut) {
        var aout = actionsOut.DiscreteActions;


        aout[0] = UnityEngine.Random.Range(0, Parameters.DimZ);
        aout[1] = UnityEngine.Random.Range(0, Parameters.DimZ);
    }

    // actions can only be relocation
    public override void OnActionReceived(ActionBuffers actions) {

        int z0 = actions.DiscreteActions[0];
        int z1 = actions.DiscreteActions[1];

        // relocation failed
        var canRelocate = bay.canRelocate(z0, z1);
        if (!canRelocate.Item1) {
            Debug.LogWarning($"failed to relocate from {z0} to {z1}");
            bool repeat = lastOperation.z0 == z0 && lastOperation.z1 == z1;
            switch (canRelocate.Item2) {
                case 0: // z0
                    lastOperation = new LastOperation(false, _0: z0, r: repeat ? lastOperation.repeatTimes + 1 : 0);
                    break;
                case 1: // z1
                    lastOperation = new LastOperation(false, _1: z1, r: repeat ? lastOperation.repeatTimes + 1 : 0);
                    break;
                case 2: // z0 and z1
                case 3:
                    lastOperation = new LastOperation(false, _0: z0, _1: z1, r: repeat ? lastOperation.repeatTimes + 1 : 0);
                    break;
                default:
                    lastOperation = new LastOperation();
                    break;
            }


            AddReward(-1 * lastOperation.repeatTimes);
            nextOperation();
            return;
        }

        // till here, the reward can be used

        var c = bay.relocate(z0, z1);
        //AddReward(-0.1f * c.relocationTimes / bay.MaxLabel);

        // step reward
        AddReward(-0.1f);


        // state blocking degree 
        //float bd = bay.BlockingDegrees.Sum();
        //AddReward(-(bd - blockingDegreeOfState) / blockingDegreeCoefficient); // if new bd smaller than old, should add positive reward

        // relocation success
        lastOperation = new LastOperation();
        nextOperation();
    }

    private void nextOperation() {
        if (bay.empty) {
            Evaluation2D.Instance.UpdateValue(maxLabel, relocationTimes);
            EndEpisode();
            return;
        }

        if (bay.canRetrieve) {
            var min = bay.min;
            relocationTimes += bay.retrieve();
            AddReward(1 - 0.01f * min.Item2); //20 * (1 - 0.01f * min.Item2) / bay.MaxLabel
            nextOperation();
            lastOperation = new LastOperation();
            return;
        }

        //Debug.Log(bay);
        RequestDecision();
    }
}












public class Container2D : IComparable {
    public readonly int priority;
    public int relocationTimes;

    public Container2D(int p) {
        relocationTimes = 0;
        priority = p;
    }

    public static bool operator >(Container2D a, Container2D b) {
        return a.priority > b.priority;
    }

    public static bool operator <(Container2D a, Container2D b) {
        return a.priority < b.priority;
    }

    public static bool operator ==(Container2D a, Container2D b) {
        if (a is null) return false;
        return a.Equals(b);
    }

    public static bool operator !=(Container2D a, Container2D b) {
        return !a.Equals(b);
    }

    public int CompareTo(object obj) {
        if (obj is Container2D c) {
            if (priority > c.priority) return 1;
            if (priority == c.priority) return 0;
            else return -1;
        }
        throw new Exception("other is not Container2D");
    }

    public override bool Equals(object obj) {
        if (obj is Container2D c) {
            return GetHashCode() == c.GetHashCode();
        }
        return false;
    }

    public override int GetHashCode() {
        return base.GetHashCode();
    }

    public override string ToString() {
        return priority.ToString();
    }
}

public class Bay {
    private Stack<Container2D>[] layout; //[z,t]
    public readonly int MaxTier;
    public readonly int DimZ;

    private int maxLabel;
    public int MaxLabel => maxLabel;

    /// <summary>
    /// this property is for observation, the first element is stack top, last is bottom
    /// </summary>
    public List<Container2D>[] Layout {
        get {
            var list = new List<Container2D>[DimZ];
            for (int i = 0; i < DimZ; i++) {
                list[i] = layout[i].ToList();
            }
            return list;
        }
    }

    public Container2D[,] LayoutAs2DArray {
        get {
            var res = new Container2D[DimZ, MaxTier];
            var layout = Layout;
            for (int z = 0; z < DimZ; z++) {
                for (int t = 0; t < DimZ; t++) {
                    res[z, t] = t < layout[z].Count ? layout[z][t] : null;
                }
            }
            return res;
        }
    }

    public int[] BlockingDegrees => layout.Select(s => BlockingDegree(s)).ToArray();

    public bool empty {
        get {
            foreach (var s in layout) {
                if (s.Count > 0) return false;
            }
            return true;
        }
    }

    /// <summary>
    /// Item1: Item, 
    /// Item2: z-index
    /// </summary>
    public (Container2D, int) min {
        get {
            Container2D m = new Container2D(int.MaxValue);
            int index = 0;
            for (int i = 0; i < layout.Length; i++) {
                if (layout[i].Count > 0) {
                    var _m = layout[i].Min();
                    if (m > _m) {
                        m = _m;
                        index = i;
                    }
                }
            }
            return (m, index);
        }
    }

    public Bay(int z, int t, int _maxLabel) {
        layout = new Stack<Container2D>[z];
        for (int i = 0; i < z; i++) {
            layout[i] = new Stack<Container2D>();
        }
        MaxTier = t;
        DimZ = z;
        maxLabel = _maxLabel;
        assignValues(generateSequence(_maxLabel));
    }

    public bool CheckDim() {
        return Layout.All(s => s.Count <= MaxTier);
    }

    public bool IndexFull(int z) {
        return layout[z].Count >= DimZ;
    }

    public bool IndexEmpty(int z) {
        return layout[z].Count <= 0;
    }

    /// <param name="z0">pick up pos</param>
    /// <param name="z1">stack pos</param>
    /// <returns>true if can relocate, the Item2 is reason--> 0: z0 empty, 1: z1 full, 2: both, 3: same index, 4: success</returns>

    public (bool, int) canRelocate(int z0, int z1) {
        (bool, int) res = (true, 4);
        if (IndexEmpty(z0)) res = (false, 0);
        if (IndexFull(z1)) res = (false, 1);
        if (IndexEmpty(z0) && IndexFull(z1)) res = (false, 2);
        if (z0 == z1) res = (false, 3);
        return res;
    }

    // check canRelocate first
    public Container2D relocate(int z0, int z1) {
        var c = layout[z0].Pop();
        c.relocationTimes++;
        layout[z1].Push(c);
        return c;
    }

    public bool stack(int z, Container2D v) {
        if (layout[z].Count == MaxTier) return false;
        layout[z].Push(v);
        return true;
    }

    public bool canRetrieve {
        get {
            var m = min;
            return layout[m.Item2].Peek() == m.Item1;
        }
    }

    /// <summary>
    /// check canRetrieve first!
    /// </summary>
    /// <returns> relocation times of this container</returns>
    public int retrieve() {
        var c = layout[min.Item2].Pop();
        return c.relocationTimes;
    }

    private int[] generateSequence(int i) {
        System.Random random = new System.Random();
        return Enumerable.Range(1, i).OrderBy(x => random.Next()).ToArray();
    }

    private void assignValues(int[] arr) {
        int i = 0;
        while (i < arr.Length) {
            int z = UnityEngine.Random.Range(0, DimZ);
            if (layout[z].Count >= MaxTier) continue;
            if (stack(z, new Container2D(arr[i]))) i++;
        }
    }

    // peak value of z-index
    public Container2D Peek(int z) {
        return layout[z].Peek();
    }


    // from https://iopscience.iop.org/article/10.1088/1742-6596/1873/1/012050/pdf
    public int BlockingDegree(Stack<Container2D> s) {
        int degree = 0;
        var list = s.Select(c => c.priority).ToList();
        list.Reverse();

        List<int> hList;
        while (list.Count > 1) {
            int truncate = list.IndexOf(list.Min());
            hList = list.GetRange(truncate, list.Count - truncate);
            if (hList.Count > 1) foreach (int x in hList) degree += hList[0] - x;
            list = list.GetRange(0, truncate);
        }

        return degree;
    }

    public override string ToString() {
        StringBuilder sb = new StringBuilder();
        foreach (var s in layout) {
            var list = s.ToList();
            list.Reverse();
            sb.Append(string.Join(", ", list) + "\n");
        }
        return sb.ToString();
    }
}

public struct LastOperation {
    public readonly bool success;
    public readonly int z0; // pick up pos
    public readonly int z1; // stack pos
    public readonly int repeatTimes;

    public LastOperation(bool s = true, int _0 = -1, int _1 = -1, int r = 0) {
        success = s;
        z0 = _0;
        z1 = _1;
        repeatTimes = r;
    }

    public static bool operator ==(LastOperation a, LastOperation b) {
        return a.success == b.success && a.z0 == b.z0 && a.z1 == b.z1;
    }

    public static bool operator !=(LastOperation a, LastOperation b) {
        return !(a == b);
    }

    public override bool Equals(object obj) {
        if (obj is LastOperation c) {
            return GetHashCode() == c.GetHashCode();
        }
        return false;
    }

    public override int GetHashCode() {
        return base.GetHashCode();
    }
}
