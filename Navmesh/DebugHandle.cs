using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;
using UnityEditor;
using UnityEditor.AddressableAssets.Settings;
using UnityEngine;
using UnityEngine.AddressableAssets;
using UnityEngine.AI;
using UnityEngine.LowLevel;

public class DebugHandle : MonoBehaviour
{
    public List<HandleCommandBuffer> hcb;
    private static DebugHandle _call;
    public static DebugHandle Call => _call;

    public UIntPtr Her;
    public void Awake()
    {
        hcb = new List<HandleCommandBuffer>();
        _call = this;
    }

    public Transform target;
    private NavMeshHit hit;
    private bool blocked = false;

    void Update()
    {
        //blocked = NavMesh.Raycast(transform.position, target.position, out hit, NavMesh.AllAreas);
        //Debug.DrawLine(transform.position, target.position, blocked ? Color.red : Color.green);
        ////DefaultWorldInitialization.Initialize
        //if (blocked)
        //    Debug.DrawRay(hit.position, Vector3.up, Color.red);


    }
    public void OnRenderObject()
    {
        Handles.DrawWireCube(Vector3.zero, Vector3.one);
        if (hcb == null)
        {
            return;
        }
        for (int i = 0; i < hcb.Count; i++)
        {
            hcb[i].DoGL();
        }
        hcb.Clear();
    }
    public static void Add(HandleCommandBuffer hcb)
    {
        if (Call==null)
        {
            return;
        }
        Call.hcb.Add(hcb);
    }


}
public abstract class HandleCommandBuffer
{
    public abstract void DoGL();
}

public class DrawDotLine : HandleCommandBuffer
{
    public DrawDotLine(Vector3[] vector3s,float size)
    {
        Vector3S = vector3s;
        Size = size;
    }

    public Vector3[] Vector3S { get; }
    public float Size { get; }

    public override void DoGL()
    {
        Handles.DrawPolyLine(Vector3S);
        //UnityEditor.Handles.DrawLines(Vector3S);
    }
}
