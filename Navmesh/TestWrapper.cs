using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using static UnityEngine.ParticleSystem;

public class TestWrapper : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public class Baker : Baker<TestWrapper>
    {
        public override void Bake(TestWrapper authoring)
        {
            AddComponent<OnlyTest>();

        }
    }
}
