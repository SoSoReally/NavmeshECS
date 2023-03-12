using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;

public class NavmeshPathTest : MonoBehaviour
{
    public Transform target;
    private NavMeshPath path;
    public NavMeshAgent agent;
    private float elapsed = 0.0f;
    void Start()
    {
        path = new NavMeshPath();
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.O))
        {
            for (int i = 0; i < 10000; i++)
            {
                Instantiate(this);
            }
        }

        if (Input.GetKeyDown(KeyCode.N))
        { 
            NavMesh.CalculatePath(transform.position, target.position, NavMesh.AllAreas, path);
           // NavMesh.SamplePosition()
            agent.SetDestination(target.position);
        }

        if (Input.GetKeyDown(KeyCode.H))
        {
            ECSManager.Init(World.DefaultGameObjectInjectionWorld);
            ECSManager.Call.World.EntityManager.CreateEntity();
            Entity x = new Entity() { Index = 110, Version = 1 };
            CreatHuman ch = new CreatHuman();
            ch.Target = x;
            ch.Execute();
        }


        //if (Input.GetKeyDown(KeyCode.M))
        //{
        //    transform.position = new Vector3(Random.Range(2f, 256), Random.Range(10f,20f), Random.Range(2f, 100f));
        //}

        //for (int i = 0; i < path.corners.Length - 1; i++)
        //    Debug.DrawLine(path.corners[i], path.corners[i + 1], Color.red);

    }
}
