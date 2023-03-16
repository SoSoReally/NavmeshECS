using System;
using System.Linq;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;
using UnityEngine.Experimental.AI;
using Random = Unity.Mathematics.Random;

namespace Game.Navmesh
{

    //note 如果导航steerstate.faile失败是否需要重新寻路？

    public static class PathSetting
    {
        private static int process = -1;
        public static int Process { get { if (process == -1) { process = Environment.ProcessorCount - 1; } return process; } } //处理器数量(-1)
        public const int MaxPathSize = 1000;           //可以获得最大路径
        public const int PreQuerySlot = 3;             //每一个QueryECS处理的QuerySlot
        public const int PreMaxIterator = 64;          //每一个QueryECS 每一帧处理的最大 路径迭代次数
        public const int InvaildQuerySlot = -1;
        [BurstCompile]
        public static Vector3 GetClosestPointOnFiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
        {
            Vector3 line_direction = line_end - line_start;
            float line_length = line_direction.magnitude;
            line_direction.Normalize();
            float project_length = Mathf.Clamp(Vector3.Dot(point - line_start, line_direction), 0f, line_length);
            return line_start + line_direction * project_length;
        }
        [BurstCompile]
        // For infinite lines:
        public static Vector3 GetClosestPointOnInfiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
        {
            return line_start + Vector3.Project(point - line_start, line_end - line_start);
        }


        [BurstCompile]
        public static bool IsPointClosestLine(in float3 start, in float3 end, in float3 point, float radiu)
        {
            if (math.all((start - end) == float3.zero)) return math.distancesq(start, point) <= radiu * radiu;
            var cloasetPoint = GetClosestPointOnFiniteLine(in start, in end, in point);
            return math.distancesq(cloasetPoint, point) <= radiu * radiu;
        }
        public static bool IsPointClosestLineSq(in float3 start, in float3 end, in float3 point, float radiusq)
        {
            var cloasetPoint = GetClosestPointOnFiniteLine(in start, in end, in point);
            return math.distancesq(cloasetPoint, point) <= radiusq;
        }

        [BurstCompile]
        public static float3 GetClosestPointOnFiniteLine(in float3 start, in float3 end, in float3 point)
        {
            var u = point - start;
            var v = end - start;
            var length = math.length(v);
            var dir = math.normalize(v);
            var p_length = math.clamp(math.dot(u, dir), 0f, length);
            return start + p_length * dir;
        }

        public static float3 GetClosestPointOnInfiniteLine(in float3 start, in float3 end, in float3 point)
        {
            var u = point - start;
            var v = end - start;
            return start + math.project(u, math.normalize(v));
            //return  math.clamp(p+start,start,end);
        }

    }

    public struct QueryECS : IDisposable
    {
        public NavMeshQuery NavMeshQuery;
        public byte UID;
        public int currentId;

        public IntPtr SlotPtr;
        public IntPtr OutputSlotPrt;
        public void Init(byte uID, NativeSlice<QuerySlot> querySlots, NativeSlice<QuerySlotOutput> querySlotOutputs)
        {
            UID = uID;
            currentId = 0;
            //QuerySlots = querySlots;
            SlotPtr = querySlots.GetUnSafeIntPtr();
            var span = SlotPtr.AsSpan<QuerySlot>(PathSetting.PreQuerySlot);
            for (int i = 0; i < span.Length; i++)
            {
                span[i].QueryECSIndex = uID;
            }
            OutputSlotPrt = querySlotOutputs.GetUnSafeIntPtr();
            NavMeshQuery = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.Persistent, ushort.MaxValue);

        }
        public void Update()
        {
            Span<QuerySlot> spanSlot = SlotPtr.AsSpan<QuerySlot>(PathSetting.PreQuerySlot);
            var spanSlotOutput = OutputSlotPrt.AsSpan<QuerySlotOutput>(PathSetting.PreQuerySlot);


            for (int i = 0; i < PathSetting.PreQuerySlot; i++)
            {
                ref var slot = ref spanSlot[currentId * PathSetting.PreQuerySlot + i];

                if (slot.state != SlotState.Ready)
                {
                    slot.state = SlotState.Failure;
                    slot.PathInfo.status = PathQueryStatus.Failure;
                    continue;
                }
                //start stage
                NavMeshLocation start = NavMeshQuery.MapLocation(slot.Path.start, Vector3.one * 100f, slot.Path.agentType, slot.Path.mask);
                NavMeshLocation end = NavMeshQuery.MapLocation(slot.Path.end, Vector3.one * 100f, slot.Path.agentType, slot.Path.mask);
                if (!NavMeshQuery.IsValid(end) || !NavMeshQuery.IsValid(start))
                {
                    slot.PathInfo.status = PathQueryStatus.Failure;
                    slot.state = SlotState.Failure;
                    continue;
                }
                var queryState = NavMeshQuery.BeginFindPath(start, end, slot.Path.mask);
                if (queryState.IsFailure())
                {
                    slot.PathInfo.status = PathQueryStatus.Failure;
                    slot.state = SlotState.Failure;
                    continue;
                }

                //update stage
                while (true)
                {
                    var state = NavMeshQuery.UpdateFindPath(PathSetting.PreMaxIterator, out var iterationsPerformed);

                    if (state.IsSuccess())
                    {
                        var endstate = NavMeshQuery.EndFindPath(out int pathSize);
                        if (pathSize > PathSetting.MaxPathSize)
                        {
                            slot.PathInfo.status = PathQueryStatus.Failure;
                            slot.state = SlotState.Failure;
                        }
                        else
                        {
                            ref var output = ref spanSlotOutput[currentId];
                            if (endstate.IsSuccess())
                            {
                                NavMeshQuery.GetPathResult(output.polygonIds);
                                var pathStatus = PathUtils.FindStraightPath(NavMeshQuery, slot.Path.start, slot.Path.end, output.polygonIds, pathSize, ref output.straightPath, ref output.straightPathFlags, ref output.vertexSide, ref slot.PathInfo.cornerCount, PathSetting.MaxPathSize);
                                if (pathStatus.IsSuccess() && slot.PathInfo.cornerCount > 1 && slot.PathInfo.cornerCount <= PathSetting.MaxPathSize)
                                {
                                    output.wayPoints.Resize(slot.PathInfo.cornerCount, NativeArrayOptions.UninitializedMemory);
                                    for (int j = 0; j < slot.PathInfo.cornerCount; j++)
                                    {
                                        output.wayPoints[j] = new WayPoint { Point = output.straightPath[j].position };
                                    }
                                    slot.PathInfo.pathSize = slot.PathInfo.cornerCount;
                                    slot.PathInfo.pathFoundToPosition = output.straightPath[slot.PathInfo.cornerCount - 1].position;
                                    slot.PathInfo.status = PathQueryStatus.Success;
                                    slot.state = SlotState.Success;
                                }
                                else
                                {
                                    slot.PathInfo.status = PathQueryStatus.Failure;
                                    slot.state = SlotState.Failure;
                                }

                            }
                            else
                            {

                                slot.PathInfo.status = PathQueryStatus.Failure;
                                slot.state = SlotState.Failure;
                            }
                        }
                        break;
                    }
                    else if (state.IsFailure())
                    {
                        slot.state = SlotState.Failure;
                        slot.PathInfo.status = PathQueryStatus.Failure;
                        break;
                    }
                    else if (state.IsInProgress())
                    {
                        continue;
                    }
                }
            }


        }

        public void Dispose()
        {
            NavMeshQuery.Dispose();
        }
    }

    public enum SlotState : int
    {
        Idle = 0,
        Ready,
        InPrograss,
        Success,
        Failure,
    }
    public struct QuerySlot : IEquatable<QuerySlot>, IDisposable
    {
        //Input 
        public Entity Entity;
        public RequestPath Path;
        public SlotState state;
        public int QueryECSIndex;
        public int Index;
        //OutPut
        public PathInfo PathInfo;
        public static void ClearState(ref QuerySlot querySlot)
        {
            querySlot.Entity = Entity.Null;
            querySlot.PathInfo = new PathInfo() { cornerCount = 0, pathFoundToPosition = float3.zero, pathSize = 0, QuerySlotIndex = PathSetting.InvaildQuerySlot, status = PathQueryStatus.Failure };
            querySlot.state = SlotState.Idle;
        }

        public bool Equals(QuerySlot other)
        {
            return this.Entity == other.Entity;
        }

        public bool IsNull()
        {
            return Entity == Entity.Null;
        }
        public void Dispose()
        {

        }
    }
    public struct QuerySlotOutput : IDisposable
    {
        [NativeDisableContainerSafetyRestriction()]
        public NativeList<WayPoint> wayPoints;
        //runtime Data
        public NativeArray<PolygonId> polygonIds;
        public int polygonIdLength;
        public NativeArray<NavMeshLocation> straightPath;
        public int straightPathLength;
        public NativeArray<StraightPathFlags> straightPathFlags;
        public int straightPathFlagsLength;
        public NativeArray<float> vertexSide;
        public int vertexSideLength;
        public void Dispose()
        {
            wayPoints.Dispose();
            polygonIds.Dispose();
            straightPath.Dispose();
            straightPathFlags.Dispose();
            vertexSide.Dispose();
        }

        public static QuerySlotOutput Default()
        {
            var Output = new QuerySlotOutput()
            {
                polygonIds = new NativeArray<PolygonId>(PathSetting.MaxPathSize * PathSetting.Process, Allocator.Persistent),
                straightPath = new NativeArray<NavMeshLocation>(PathSetting.MaxPathSize * PathSetting.Process, Allocator.Persistent),
                straightPathFlags = new NativeArray<StraightPathFlags>(PathSetting.MaxPathSize * PathSetting.Process, Allocator.Persistent),
                vertexSide = new NativeArray<float>(PathSetting.MaxPathSize * PathSetting.Process, Allocator.Persistent),
                wayPoints = new NativeList<WayPoint>(64, Allocator.Persistent)
            };
            return Output;
        }

        public static void Clear(ref QuerySlotOutput querySlotOutput)
        {
            querySlotOutput.wayPoints.Length = 0;
            querySlotOutput.polygonIdLength = 0;
            querySlotOutput.straightPathLength = 0;
            querySlotOutput.straightPathFlagsLength = 0;
            querySlotOutput.vertexSideLength = 0;
        }
    }
    /// <summary>
    /// 路径请求
    /// </summary>
    public struct RequestPath : IComponentData, IEnableableComponent
    {
        public Entity myself;
        public float3 start;
        public float3 end;
        public int agentIndex;
        public int agentType;
        public int mask;
        public uint uid;
        public float ThresholdDistance;
    }

    /// <summary>
    /// 路径请求的信息
    /// </summary>
    public struct PathInfo : IComponentData
    {
        public PathQueryStatus status;
        public int pathSize;
        public float3 pathFoundToPosition;
        public int cornerCount;
        public int QuerySlotIndex;
    }


    public struct NavAgentPathSteer : IComponentData, IEnableableComponent
    {
        public int TargetPointIndex;
        public float3 start; //段落起点
        public float PickWayPointDistance;
        public StreerState streerState;

        public static void Start(ref NavAgentPathSteer navAgentPathSteer)
        {
            navAgentPathSteer.start = float3.zero;
            navAgentPathSteer.TargetPointIndex = 0;
            navAgentPathSteer.streerState = StreerState.Start;
        }
    }
    public enum StreerState : int
    {
        Start,
        Run,
        Faile,
        Over,
    }

    /// <summary>
    /// </summary>

    public struct NavAgentComponent : IComponentData, IEnableableComponent
    {
        public float speed;
        public float angleSpeed;
        public float stopDistance;
        public float3 exetern;
        public int areaMask;
        public int agentType;
        public float externLength;
    }
    public struct NavAgentTransform : IComponentData
    {
        public float3 position;
        public quaternion rotation;
    }

    public struct NavAgentLocation : IComponentData
    {
        public NavMeshLocation NavMeshLocation;
    }
    public struct WayPoint : IBufferElementData
    {
        public float3 Point;
    }


    public partial class NavmeshSystem : ComponentSystemGroup {
    
    
    }

    public static class NavmeshUtility
    {
        public static void Setup(Entity e, EntityManager entityManager)
        {
            entityManager.AddComponentData<NavAgentComponent>(e, new NavAgentComponent() { angleSpeed = 30f, speed = 2f, stopDistance = 0.1f, agentType = 0, areaMask = -1, exetern = 1f });
            entityManager.AddComponentData<NavAgentTransform>(e, new NavAgentTransform() { position = Unity.Mathematics.float3.zero, rotation = quaternion.identity });
            entityManager.AddComponentData<NavAgentPathSteer>(e, new NavAgentPathSteer() { TargetPointIndex = 0, PickWayPointDistance = 0.2f });
            entityManager.AddComponentData<NavAgentLocation>(e, new NavAgentLocation() { });
            entityManager.AddComponentData<RequestPath>(e, default);
            entityManager.AddComponentData<PathInfo>(e, default);
            entityManager.AddBuffer<WayPoint>(e);
            entityManager.SetComponentEnabled<RequestPath>(e, false);
            entityManager.SetComponentEnabled<NavAgentComponent>(e, false);
        }
        public static void Setup(NativeArray<Entity> e, EntityManager entityManager)
        {
            FixedList128Bytes<ComponentType> comList = new FixedList128Bytes<ComponentType>
            {
                Length = 7
            };
            comList.Clear();
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentComponent>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentTransform>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentPathSteer>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentLocation>());
            comList.AddNoResize(ComponentType.ReadWrite<RequestPath>());
            comList.AddNoResize(ComponentType.ReadWrite<PathInfo>());
            comList.AddNoResize(ComponentType.ReadWrite<WayPoint>());
            entityManager.AddComponent(e, new ComponentTypeSet(comList));
            for (int i = 0; i < e.Length; i++)
            {
                entityManager.SetComponentData<NavAgentComponent>(e[i], new NavAgentComponent() { angleSpeed = 30f, speed = 2f, stopDistance = 0.1f, agentType = 0, areaMask = -1, exetern = 1f });
                entityManager.SetComponentData<NavAgentTransform>(e[i], new NavAgentTransform() { position = Unity.Mathematics.float3.zero, rotation = quaternion.identity });
                entityManager.SetComponentData<NavAgentPathSteer>(e[i], new NavAgentPathSteer() { TargetPointIndex = 0, PickWayPointDistance = 0.2f });
                entityManager.SetComponentData<NavAgentLocation>(e[i], new NavAgentLocation() { });
                entityManager.SetComponentEnabled<RequestPath>(e[i], false);
                entityManager.SetComponentEnabled<NavAgentComponent>(e[i], false);
            }

        }
        public static void Remove(NativeArray<Entity> entities, EntityManager entityManager)
        {
            entityManager.RemoveComponent<NavAgentComponent>(entities);
            entityManager.RemoveComponent<NavAgentTransform>(entities);
            entityManager.RemoveComponent<NavAgentPathSteer>(entities);
            entityManager.RemoveComponent<NavAgentLocation>(entities);
            entityManager.RemoveComponent<RequestPath>(entities);
            entityManager.RemoveComponent<PathInfo>(entities);
            entityManager.RemoveComponent<WayPoint>(entities);
        }
        public static void Remove(Entity entities, EntityManager entityManager)
        {
            entityManager.RemoveComponent<NavAgentComponent>(entities);
            entityManager.RemoveComponent<NavAgentTransform>(entities);
            entityManager.RemoveComponent<NavAgentPathSteer>(entities);
            entityManager.RemoveComponent<NavAgentLocation>(entities);
            entityManager.RemoveComponent<RequestPath>(entities);
            entityManager.RemoveComponent<PathInfo>(entities);
            entityManager.RemoveComponent<WayPoint>(entities);
        }
    }

#if UNITY_EDITOR
    public struct OnlyTest : IComponentData
    {
        public int INDEX;
    }

    public readonly partial struct TestAspect : IAspect
    {
        public readonly RefRO<OnlyTest> refROTest;
    }

    [UpdateInGroup(typeof(NavmeshSystem), OrderFirst = true)]
    public partial class PathTestSystem : SystemBase
    {
        public Entity Entity;
        public EntityQuery entityQuery;
        public Random RadiuRandom;

        public NativeArray<Entity> EntiyNative;


        public static Entity Test;
        public static bool Boll;
        protected override void OnCreate()
        {
            RadiuRandom = Random.CreateFromIndex(10);
            Test = EntityManager.CreateEntity();
            EntityManager.AddComponentData<OnlyTest>(Test, new OnlyTest() { INDEX = 10 });
        }
        protected override void OnUpdate()
        {
            if (Boll)
            {
                Debug.Log(UnityEngine.Time.frameCount);
                Boll = false;
            }
            if (Input.GetKeyDown(KeyCode.Q))
            {
                EntityManager.SetComponentData<OnlyTest>(Test, new OnlyTest() { INDEX = (int)SystemAPI.Time.ElapsedTime });
            }

            if (Input.GetKeyDown(KeyCode.I))
            {
                EntityCommandBuffer ecb = new EntityCommandBuffer(Allocator.Temp);

                Entities.ForEach((Entity e, in OnlyTest only) =>
                {
                    ecb.AddComponent<NavAgentComponent>(e, new NavAgentComponent() { angleSpeed = 30f, speed = 2f, stopDistance = 0.1f, agentType = 0, areaMask = -1, exetern = 1f });
                    ecb.AddComponent<NavAgentTransform>(e, new NavAgentTransform() { position = Unity.Mathematics.float3.zero, rotation = quaternion.identity });
                    ecb.AddComponent<NavAgentPathSteer>(e, new NavAgentPathSteer() { TargetPointIndex = 0, PickWayPointDistance = 0.2f });
                    ecb.AddComponent<NavAgentLocation>(e, new NavAgentLocation() { });

                    ecb.AddComponent<RequestPath>(e);
                    ecb.AddComponent<PathInfo>(e);
                    ecb.AddBuffer<WayPoint>(e);
                    ecb.SetComponentEnabled<RequestPath>(e, false);
                    ecb.SetComponentEnabled<NavAgentComponent>(e, false);
                }).WithStoreEntityQueryInField(ref entityQuery).Run();
                ecb.Playback(EntityManager);
                ecb.Dispose();
                var t = entityQuery.ToEntityArray(Allocator.Temp);
                var min = math.min(1, t.Length);
                for (int i = 0; i < min; i++)
                {
                    EntiyNative = EntityManager.Instantiate(t[0], 1000, Allocator.Persistent);
                }
            }

            if (Input.GetKeyDown(KeyCode.Y))
            {
                Debug.Log(EntiyNative.Length);
                Debug.Log(EntityManager.HasComponent<RequestPath>(EntiyNative[0]));
                for (int i = 0; i < EntiyNative.Length; i++)
                {
                    var Entity = EntiyNative[i];
                    EntityManager.SetComponentEnabled<RequestPath>(Entity, true);
                    //var pos = EntityManager.GetComponentData<Translation>(Entity).Value;
                    //UnsafeStream stream = new UnsafeStream();
                    //stream.AsWriter().Write(EntiyNative)
                    //System.Runtime.InteropServices.MemoryMarshal.AsBytes(MemoryMarshal.CreateReadOnlySpan(ref pos,1));
                    var pos = float3.zero;
                    if (EntityManager.HasComponent<RequestPath>(Entity))
                    {
                        EntityManager.SetComponentData(Entity,
                        new RequestPath()
                        {
                            agentIndex = 0,
                            agentType = 0,
                            end = RadiuRandom.NextFloat3(new float3(-30f, 0f, -30f), new float3(30f, 0f, 30f)),
                            mask = -1,
                            myself = Entity,
                            start = pos,
                            uid = 1,
                            ThresholdDistance = EntityManager.GetComponentData<NavAgentComponent>(Entity).stopDistance,
                        });
                    }
                    else
                    {
                        Debug.Log(i);
                    }

                }


            }
            new DrawNavPathJob().Run();
        }
        protected override void OnDestroy()
        {
            if (EntiyNative.IsCreated)
            {
                EntiyNative.Dispose();
            }
            base.OnDestroy();
        }
        public partial struct DrawNavPathJob : IJobEntity
        {
            public void Execute(in OnlyTest onlyTest, in PathInfo path, in DynamicBuffer<WayPoint> wayPoints)
            {
                if (path.status != PathQueryStatus.Success)
                {
                    return;
                }
                var read = wayPoints.ToNativeArray(Allocator.Temp).ToArray();
                DebugHandle.Add(new DrawDotLine(read.Select<WayPoint, Vector3>((a) => { return a.Point; }).ToArray(), 0.5f));
            }
        }
    }
#endif
    /// <summary>
    /// 这个系统负责寻找路径
    /// </summary>
    [UpdateInGroup(typeof(NavmeshSystem))]
    public partial class FindPathSystem : SystemBase
    {

        public NativeArray<QueryECS> QueryECSArray;
        public NativeArray<QuerySlot> QuerySlotArray;
        public NativeArray<QuerySlotOutput> QuerySlotOutputArray;

        public EntityQuery RequestPathQuery_1;

        EndSimulationEntityCommandBufferSystem endBuffer;
        public ComponentLookup<RequestPath> RequestPathLookup;
        public ComponentLookup<PathInfo> PathInfoLookup;
        public ComponentLookup<NavAgentPathSteer> NavAgentPathSteerLookup;
        public BufferLookup<WayPoint> WayPointLookup;
        //public TransformAspect.Lookup TransformLookup;

        protected override void OnCreate()
        {
            endBuffer = World.GetExistingSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var allSlot = PathSetting.Process * PathSetting.PreQuerySlot;
            QueryECSArray = new NativeArray<QueryECS>(PathSetting.Process, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            QuerySlotArray = new NativeArray<QuerySlot>(allSlot, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            QuerySlotOutputArray = new NativeArray<QuerySlotOutput>(allSlot, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            for (int i = 0; i < QuerySlotArray.Length; i++)
            {
                var value = QuerySlotArray[i];
                value.Index = i;
                QuerySlot.ClearState(ref value);
                QuerySlotOutputArray[i] = QuerySlotOutput.Default();
            }


            var QueryECSArraySpan = QueryECSArray.AsSpan();
            for (int i = 0; i < PathSetting.Process; i++)
            {
                var start = i * PathSetting.PreQuerySlot;
                var length = PathSetting.PreQuerySlot;
                QueryECSArraySpan[i].Init((byte)i, QuerySlotArray.Slice(start, length), QuerySlotOutputArray.Slice(start, length));
            }
            RequestPathQuery_1 = GetEntityQuery(typeof(RequestPath), typeof(WayPoint), typeof(PathInfo));


            RequestPathLookup = GetComponentLookup<RequestPath>(false);
            PathInfoLookup = GetComponentLookup<PathInfo>(false);
            NavAgentPathSteerLookup = GetComponentLookup<NavAgentPathSteer>(false);
            WayPointLookup = GetBufferLookup<WayPoint>(false);
            //TransformLookup = new TransformAspect.Lookup();
            //EntityManager.SetComponentData
        }

        protected override void OnUpdate()
        {
            RequestPathLookup.Update(ref CheckedStateRef);
            PathInfoLookup.Update(ref CheckedStateRef);
            NavAgentPathSteerLookup.Update(ref CheckedStateRef);
            WayPointLookup.Update(ref CheckedStateRef);
            //need dispose
            var EntityList = RequestPathQuery_1.ToEntityListAsync(Allocator.TempJob, Dependency, out var ArrayJob);

            //var requestPathList = RequestPathQuery_1.ToComponentDataListAsync<RequestPath>(Allocator.TempJob, Dependency, out var chunkArrayJob);
            //var cb = endBuffer.CreateCommandBuffer();
            //EntityCommandBuffer = new EntityCommandBuffer()
            var check = new CheckAndEntryQuerySlotJob() { EntityList = EntityList, querySlots = QuerySlotArray, PathInfoLookup = PathInfoLookup, RequestPathLookup = RequestPathLookup, querySlotOutputs = QuerySlotOutputArray.GetUnSafeIntPtr() }.Schedule(ArrayJob);
            var ecsjob = new QueryECSJob() { QueryECSPtr = QueryECSArray.GetUnSafeIntPtr(), Process = PathSetting.Process }.Schedule(PathSetting.Process, 1, check);
            Dependency = new GetQuerySlotOutputJob() { NavAgentPathSteerLookup = NavAgentPathSteerLookup, PathInfoLookup = PathInfoLookup, WayPointLookup = WayPointLookup, QuerySlotsArray = QuerySlotArray, QuerySlotsOutputArray = QuerySlotOutputArray.GetUnSafeIntPtr() }.Schedule(ecsjob);
            endBuffer.AddJobHandleForProducer(Dependency);
            EntityList.Dispose(Dependency);
            //CompleteDependency();
        }
        protected override void OnDestroy()
        {
            CompleteDependency();
            for (int i = 0; i < QuerySlotOutputArray.Length; i++)
            {
                QuerySlotOutputArray[i].Dispose();
            }
            QuerySlotArray.Dispose();
            QueryECSArray.Dispose();
            QuerySlotOutputArray.Dispose();

            base.OnDestroy();
        }


        [BurstCompile]
        public partial struct CheckAndEntryQuerySlotJob : IJob
        {
            public ComponentLookup<RequestPath> RequestPathLookup;
            public ComponentLookup<PathInfo> PathInfoLookup;
            public NativeList<Entity> EntityList;
            public NativeArray<QuerySlot> querySlots;

            [NativeDisableUnsafePtrRestriction()]
            public IntPtr querySlotOutputs;
            public void Execute()
            {
                if (EntityList.Length <= 0)
                {
                    return;
                }
                var spanQuerySlot = querySlots.AsSpan();
                var min = math.min(EntityList.Length, spanQuerySlot.Length);
                var querySlotOutput = querySlotOutputs.AsSpan<QuerySlotOutput>(querySlots.Length);
                for (int i = 0; i < min; i++)
                {
                    var e = EntityList[i];
                    RequestPathLookup.SetComponentEnabled(e, false);
                    ref readonly var requestPath = ref RequestPathLookup.GetRefRO(e).ValueRO;
                    ref var pathInfoRW = ref PathInfoLookup.GetRefRW(EntityList[i], false).ValueRW;
                    //无效寻路
                    if (math.distancesq(requestPath.start, requestPath.end) < requestPath.ThresholdDistance * requestPath.ThresholdDistance)
                    {
                        continue;

                    }
                    ref readonly var request = ref RequestPathLookup.GetRefRO(EntityList[i]).ValueRO;
                    ref var slot = ref spanQuerySlot[i];
                    slot.Path = request;
                    slot.Entity = request.myself;
                    slot.state = SlotState.Ready;
                    QuerySlotOutput.Clear(ref querySlotOutput[i]);
                    RequestPathLookup.SetComponentEnabled(request.myself, false);
                    var rw = PathInfoLookup.GetRefRWOptional(slot.Entity, false);
                    rw.ValueRW = new PathInfo() { cornerCount = 0, pathFoundToPosition = 0f, pathSize = 0, QuerySlotIndex = i, status = PathQueryStatus.InProgress };
                }
            }
        }
        [BurstCompile]
        public struct QueryECSJob : IJobParallelFor
        {
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECSPtr;

            [ReadOnly]
            public int Process;
            public void Execute(int index)
            {
                QueryECSPtr.AsSpan<QueryECS>(Process)[index].Update();
            }
        }
        [BurstCompile]
        public struct GetQuerySlotOutputJob : IJob
        {

            public NativeArray<QuerySlot> QuerySlotsArray;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QuerySlotsOutputArray;
            //public ComponentLookup<RequestPath> RequestPathLookup;
            public ComponentLookup<PathInfo> PathInfoLookup;
            public ComponentLookup<NavAgentPathSteer> NavAgentPathSteerLookup;
            public BufferLookup<WayPoint> WayPointLookup;
            public void Execute()
            {
                var spanSlotOutput = QuerySlotsOutputArray.AsReadOnlySpan<QuerySlotOutput>(QuerySlotsArray.Length);
                var spanSlot = QuerySlotsArray.AsSpan();


                for (int i = 0; i < spanSlot.Length; i++)
                {
                    //if (span[i].HasRequest)
                    //{
                    //    continue;
                    //}
                    ref var item = ref spanSlot[i];
                    if (item.Entity == Entity.Null)
                    {
                        return;
                    }


                    switch (item.state)
                    {
                        case SlotState.Success:

                            var wayPointDynamic = WayPointLookup[item.Entity];
                            wayPointDynamic.Clear();

                            var PathInfo = PathInfoLookup.GetRefRW(item.Entity, false);
                            PathInfo.ValueRW = item.PathInfo;
                            //ecb.SetComponent<PathInfo>(item.Entity, item.PathInfo);
                            //wayPointDynamic.AddRange(spanSlotOutput[i].wayPoints.AsArray());
                            var waypointSpan = spanSlotOutput[i].wayPoints.AsSpan();
                            var NavAgentPathSteerRW = NavAgentPathSteerLookup.GetRefRW(item.Entity, false);
                            NavAgentPathSteer.Start(ref NavAgentPathSteerRW.ValueRW);
                            wayPointDynamic.UnSafeAddSpan(waypointSpan);
                            //wayPointDynamic.AddRange(waypointSpan.AsArray());
                            QuerySlot.ClearState(ref item);
                            break;
                        case SlotState.Failure:
                            var wayPointDynamicFaile = WayPointLookup[item.Entity];
                            wayPointDynamicFaile.Clear();
                            var PathInfoFail = PathInfoLookup.GetRefRW(item.Entity, false);
                            PathInfoFail.ValueRW = item.PathInfo;
                            QuerySlot.ClearState(ref item);
                            break;
                        default:
                            break;

                    }

                }
            }
        }


    }


    /// <summary>
    /// 这个系统负责更新路径寻路 到NavTransform组件中
    /// 需要使用其他系统读取Transform 到 NavTransform 中  以及 写入NavTransform 到Transform 中
    /// </summary>
    [UpdateInGroup(typeof(NavmeshSystem))]
    [UpdateAfter(typeof(FindPathSystem))]
    public partial class NavmeshPathUpdateSystem : SystemBase
    {
        public EndSimulationEntityCommandBufferSystem EndBufferSystem;
        public NavMeshQuery OnlyLocation;


        public EntityQuery CheckNavAgentComponentEnableEntityQuery_1;

        public ComponentTypeHandle<NavAgentComponent> NavAgentComponentHandle;
        public ComponentTypeHandle<NavAgentTransform> NavTransformTypeHandle;
        //public EntityTypeHandle EntityTypeHandle;
        public ComponentTypeHandle<PathInfo> PathInfoHandle;
        public BufferTypeHandle<WayPoint> WayPointHandle;
        public ComponentTypeHandle<NavAgentComponent> EnableNavAgentComponent;
        //public TransformAspect.TypeHandle TransformHandle;
        public ComponentTypeHandle<NavAgentPathSteer> NavAgentPathSteerHandle;
        public ComponentTypeHandle<NavAgentLocation> NavAgentLocationHandle;





        public EntityQuery SyncTransfromEntityQuery_2;




        public BufferTypeHandle<WayPoint> WayPointHandle2;
        public ComponentTypeHandle<NavAgentComponent> NavAgentComponentHandle2;
        //[ReadOnly] public ComponentTypeHandle<PathInfo> PathInfoHandle;
        public TransformAspect.TypeHandle TransformAspectTypeHandle2;
        public ComponentTypeHandle<NavAgentPathSteer> NavAgentPathSteerHandle2;
        public ComponentTypeHandle<NavAgentTransform> NavAgentTransformHandle2;
        public ComponentTypeHandle<NavAgentLocation> NavAgentLocationHandle2;


        protected override void OnCreate()
        {
            EndBufferSystem = this.World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            OnlyLocation = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.Persistent);


            NativeList<ComponentType> querytemp = new NativeList<ComponentType>(8, Allocator.Temp);

            Span<ComponentType> span = stackalloc ComponentType[] {
                ComponentType.ReadWrite<NavAgentComponent>(),
                ComponentType.ReadWrite<NavAgentLocation>(),
                ComponentType.ReadOnly<NavAgentTransform>(),
                ComponentType.ReadOnly<WayPoint>(),
                ComponentType.ReadOnly<PathInfo>(),
                ComponentType.ReadOnly<NavAgentPathSteer>(),
            };

            querytemp.AddRange(span.AsNativeArray());
            CheckNavAgentComponentEnableEntityQuery_1 = GetEntityQuery(new EntityQueryBuilder(Allocator.Temp)
                .WithAll(ref querytemp)
                .WithOptions(EntityQueryOptions.IgnoreComponentEnabledState));


            NavAgentComponentHandle = GetComponentTypeHandle<NavAgentComponent>(false);
            NavAgentLocationHandle = GetComponentTypeHandle<NavAgentLocation>(false);
            NavTransformTypeHandle = GetComponentTypeHandle<NavAgentTransform>(true);
            // EntityTypeHandle = GetEntityTypeHandle();
            PathInfoHandle = GetComponentTypeHandle<PathInfo>(true);
            WayPointHandle = GetBufferTypeHandle<WayPoint>(true);
            //TransformHandle = new TransformAspect.TypeHandle(ref CheckedStateRef, true);
            NavAgentPathSteerHandle = GetComponentTypeHandle<NavAgentPathSteer>(true);


            querytemp.Clear();

            Span<ComponentType> span2 = stackalloc ComponentType[]
            {
                ComponentType.ReadOnly<NavAgentComponent>(),
                ComponentType.ReadWrite<NavAgentTransform>(),
                ComponentType.ReadOnly<WayPoint>(),
                ComponentType.ReadOnly<PathInfo>(),
                ComponentType.ReadWrite<NavAgentPathSteer>(),
                ComponentType.ReadWrite<NavAgentLocation>(),
            };
            querytemp.AddRange(span2.AsNativeArray());
            SyncTransfromEntityQuery_2 = GetEntityQuery(new EntityQueryBuilder(Allocator.Temp).WithAll(ref querytemp));

            WayPointHandle2 = GetBufferTypeHandle<WayPoint>(true);
            NavAgentComponentHandle2 = GetComponentTypeHandle<NavAgentComponent>(true);
            //[ReadOnly] public ComponentTypeHandle<PathInfo> PathInfoHandle;
            TransformAspectTypeHandle2 = new TransformAspect.TypeHandle(ref CheckedStateRef, false);
            NavAgentPathSteerHandle2 = GetComponentTypeHandle<NavAgentPathSteer>(false);
            NavAgentTransformHandle2 = GetComponentTypeHandle<NavAgentTransform>(false);
            NavAgentLocationHandle2 = GetComponentTypeHandle<NavAgentLocation>(false);
        }

        protected override void OnUpdate()
        {
            NavAgentComponentHandle.Update(this);
            NavTransformTypeHandle.Update(this);
            //EntityTypeHandle.Update(this);
            PathInfoHandle.Update(this);
            WayPointHandle.Update(this);
            //TransformHandle.Update(ref CheckedStateRef);
            EnableNavAgentComponent.Update(this);
            NavAgentPathSteerHandle.Update(this);
            NavAgentLocationHandle.Update(this);
            var deltaTime = SystemAPI.Time.DeltaTime;
            //--------stage1
            Dependency = new CheckEnableNavAgentWithLocation()
            {
                TimeDelta = deltaTime,
                NavAgentComponentHandle = NavAgentComponentHandle,
                NavTransformTypeHandle = NavTransformTypeHandle,
                //EntityTypeHandle = EntityTypeHandle,
                PathInfoHandle = PathInfoHandle,
                WayPointHandle = WayPointHandle,
                //TransformHandle = TransformHandle,
                NavAgentPathSteerHandle = NavAgentPathSteerHandle,
                NavAgentLocationHandle = NavAgentLocationHandle,
                NavMeshQuery = OnlyLocation
            }.ScheduleParallel(CheckNavAgentComponentEnableEntityQuery_1, Dependency);

            //--------stage2
            //Dependency = new CheckPathAndSyncLocationJob() { navMeshQuery = OnlyLocation }.ScheduleParallel(Dependency);


            //--------stage3
            NavAgentComponentHandle2.Update(this);
            NavAgentLocationHandle2.Update(this);
            NavAgentPathSteerHandle2.Update(this);
            NavAgentTransformHandle2.Update(this);
            TransformAspectTypeHandle2.Update(ref CheckedStateRef);
            WayPointHandle2.Update(this);
            Dependency = new UpdateNavAgentTransformJob()
            {
                TimeDeltaTime = deltaTime,
                navMeshQuery = OnlyLocation,
                NavAgentComponentHandle = NavAgentComponentHandle2,
                NavAgentLocationHandle = NavAgentLocationHandle2,
                NavAgentPathSteerHandle = NavAgentPathSteerHandle2,
                NavAgentTransformHandle = NavAgentTransformHandle2,
                //TransformAspectTypeHandle = TransformAspectTypeHandle2,
                WayPointHandle = WayPointHandle2,
            }.ScheduleParallel(SyncTransfromEntityQuery_2, Dependency);
            EndBufferSystem.AddJobHandleForProducer(Dependency);

        }

        protected override void OnDestroy()
        {
            OnlyLocation.Dispose();
            base.OnDestroy();
        }



        public void SetEnable(Entity e,bool boolean)
        {
            World.EntityManager.SetComponentEnabled<NavAgentComponent>(e, boolean);
        }
        public static ComponentType[] RequiredComponents => new ComponentType[]{
        typeof(NavAgentComponent),
        typeof(NavAgentTransform),
        typeof(NavAgentPathSteer),
        typeof(NavAgentLocation),
        typeof(RequestPath),
        typeof(PathInfo),
        typeof(WayPoint) };
        //public ComponentType[] DepeondComponets => new ComponentType[] { };
        /// <summary>
        /// 检查开启组件，坐标定位
        /// 
        /// </summary>
        [BurstCompile]
        public partial struct CheckEnableNavAgentWithLocation : IJobChunk
        {
            public ComponentTypeHandle<NavAgentComponent> NavAgentComponentHandle;
            public ComponentTypeHandle<NavAgentLocation> NavAgentLocationHandle;
            [ReadOnly] public ComponentTypeHandle<NavAgentTransform> NavTransformTypeHandle;
            //[ReadOnly] public EntityTypeHandle EntityTypeHandle;
            [ReadOnly] public ComponentTypeHandle<PathInfo> PathInfoHandle;
            [ReadOnly] public BufferTypeHandle<WayPoint> WayPointHandle;
            //[ReadOnly] public TransformAspect.TypeHandle TransformHandle;
            [ReadOnly] public ComponentTypeHandle<NavAgentPathSteer> NavAgentPathSteerHandle;


            [ReadOnly] public NavMeshQuery NavMeshQuery;
            [ReadOnly] public float TimeDelta;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                EnabledMask enabledMask = chunk.GetEnabledMask(ref NavAgentComponentHandle);
                //if (!enabledMask.EnableBit.IsValid)
                //{
                //    return;
                //}
                var navAgentComArray = chunk.GetNativeArray(ref NavAgentComponentHandle).AsReadOnlySpan();
                var navTransformArray = chunk.GetNativeArray(ref NavTransformTypeHandle).AsReadOnlySpan();
                //var entityArray = chunk.GetNativeArray(EntityTypeHandle).AsReadOnlySpan();
                var pathInfoArray = chunk.GetNativeArray(ref PathInfoHandle).AsReadOnlySpan();
                var wayPoint = chunk.GetBufferAccessor(ref WayPointHandle);
                //var transformArray = TransformHandle.Resolve(chunk);
                var navAgentLocationArray = chunk.GetNativeArray(ref NavAgentLocationHandle).AsSpan();
                var navAgentPathSteerArray = chunk.GetNativeArray(ref NavAgentPathSteerHandle).AsReadOnlySpan();
                for (int i = 0; i < chunk.Count; i++)
                {
                    ref readonly var itemNavAgentCom = ref navAgentComArray[i];
                    ref readonly var itemNavTransform = ref navTransformArray[i];
                    //ref readonly var itemEntity = ref entityArray[i];
                    ref readonly var itemPathInfo = ref pathInfoArray[i];
                    var c = wayPoint[i];
                    int waypointLength = c.Length;
                    //var transfrom = transformArray[i];
                    ref var navAgentLocation = ref navAgentLocationArray[i];
                    ref readonly var itemNavPathIndex = ref navAgentPathSteerArray[i];

                    bool isShouldEnable = true;
                    var stopdistance = (itemNavAgentCom.stopDistance + itemNavAgentCom.speed * TimeDelta);
                    //1
                    if (
                        //itemPathInfo.status != PathQueryStatus.Success
                        //||
                        waypointLength <= 0
                        || itemNavPathIndex.streerState == StreerState.Faile
                        || itemNavPathIndex.streerState == StreerState.Over
                        || itemNavPathIndex.TargetPointIndex >= waypointLength
                        || math.distancesq(itemPathInfo.pathFoundToPosition, itemNavTransform.position) <= stopdistance * stopdistance
                        )
                    {
                        isShouldEnable = false;
                    }

                    if (isShouldEnable)
                    {
                        ///定位在寻路网格上
                        navAgentLocation.NavMeshLocation = NavMeshQuery.MapLocation(itemNavTransform.position, itemNavAgentCom.exetern, itemNavAgentCom.agentType);
                        isShouldEnable = NavMeshQuery.IsValid(navAgentLocation.NavMeshLocation);
#if UNITY_EDITOR
                        Debug.Assert(isShouldEnable, "location fail");
#endif
                    }

                    if (isShouldEnable != enabledMask[i])
                    {
                        enabledMask[i] = isShouldEnable;
                    }

                }
            }
        }

        //[BurstCompile]
        //public partial struct CheckPathAndSyncLocationJob : IJobEntity
        //{
        //    [ReadOnly] public NavMeshQuery navMeshQuery;

        //    public void Execute(in DynamicBuffer<WayPoint> wayPoint, in NavAgentComponent com, ref NavAgentTransform navAgentTransform, ref NavAgentPathSteer navAgentPathSteer,in NavAgentLocation navAgentLocation)
        //    {
        //        navAgentTransform.position = navAgentLocation.NavMeshLocation.position;

        //        //没有目标点
        //        //  return
        //        //在路径上
        //        //  return
        //        //从列表顶部寻找可到达的WayPoint
        //        //  设置巡航steer

        //        if (wayPoint.Length <= 1 || navAgentPathSteer.TargetPointIndex >= wayPoint.Length) return;

        //        var wayPointSpan = wayPoint.AsReadOnlySpan();
        //        var position = navAgentLocation.NavMeshLocation.position;


        //        ref readonly var targetWaypoint = ref wayPointSpan[navAgentPathSteer.TargetPointIndex];
        //        if (PathSetting.IsPointClosestLine(in navAgentPathSteer.start, in targetWaypoint.Point, navAgentTransform.position, navAgentPathSteer.PickWayPointDistance, out var OnPoint))
        //        {

        //            if ( math.distancesq( position ,targetWaypoint.Point)< com.stopDistance* com.stopDistance)
        //            {
        //                navAgentPathSteer.TargetPointIndex++;
        //                navAgentPathSteer.start = position;
        //            }
        //            return;
        //        }

        //        for (int i = wayPointSpan.Length - 1; i < 0; i--)
        //        {
        //            var state = navMeshQuery.Raycast(out var navMeshHit, navAgentLocation.NavMeshLocation, wayPointSpan[i].Point, com.areaMask);
        //            if ((state & PathQueryStatus.Success) != 0)
        //            {
        //                if (navMeshHit.hit)
        //                {
        //                    navAgentPathSteer.start = navAgentTransform.position;
        //                    navAgentPathSteer.TargetPointIndex = i;
        //                    break;
        //                }
        //            }
        //        }
        //    }
        //}
        [BurstCompile]
        public partial struct UpdateNavAgentTransformJob : IJobChunk
        {

            [ReadOnly] public float TimeDeltaTime;
            [ReadOnly] public NavMeshQuery navMeshQuery;
            [ReadOnly] public BufferTypeHandle<WayPoint> WayPointHandle;
            [ReadOnly] public ComponentTypeHandle<NavAgentComponent> NavAgentComponentHandle;
            public ComponentTypeHandle<NavAgentPathSteer> NavAgentPathSteerHandle;
            public ComponentTypeHandle<NavAgentTransform> NavAgentTransformHandle;
            public ComponentTypeHandle<NavAgentLocation> NavAgentLocationHandle;
            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                ChunkEntityEnumerator chunkEntityEnumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                var BufferWayPointArray = chunk.GetBufferAccessor(ref WayPointHandle);
                var NavAgentComponentArray = chunk.GetNativeArray(ref NavAgentComponentHandle).AsReadOnlySpan();
                var NavAgentTransformArray = chunk.GetNativeArray(ref NavAgentTransformHandle).AsSpan();
                //var PathInfoArray = chunk.GetNativeArray(PathInfoHandle).AsReadOnlySpan();
                var NavAgentPathSteerArray = chunk.GetNativeArray(ref NavAgentPathSteerHandle).AsSpan();
                var NavAgentLocationArray = chunk.GetNativeArray(ref NavAgentLocationHandle).AsSpan();





                NativeArray<NavMeshLocation> locations = new NativeArray<NavMeshLocation>(chunk.Count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<int> IndexArray = new NativeArray<int>(chunk.Count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<Vector3> TargetArray = new NativeArray<Vector3>(chunk.Count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                NativeArray<int> AreaMask = new NativeArray<int>(chunk.Count, Allocator.Temp, NativeArrayOptions.UninitializedMemory);



                int count = 0;
                while (chunkEntityEnumerator.NextEntityIndex(out int nextIndex))
                {

                    ref var navAgentPathSteer = ref NavAgentPathSteerArray[nextIndex];

                    ref var navAgentLocation = ref NavAgentLocationArray[nextIndex];

                    ref var navAgentTransform = ref NavAgentTransformArray[nextIndex];

                    var waypointbuffer = BufferWayPointArray[nextIndex];

                    ref readonly var navAgentCom = ref NavAgentComponentArray[nextIndex];
                    //var pathInfo = PathInfoArray[nextIndex];

                    var wayPointBufferSpan = waypointbuffer.AsReadOnlySpan();
                    navAgentTransform.position = navAgentLocation.NavMeshLocation.position;
                    switch (navAgentPathSteer.streerState)
                    {
                        case StreerState.Start:
                            navAgentPathSteer.streerState = StreerState.Run;
                            break;
                        case StreerState.Run: break;
                        case StreerState.Faile: continue;
                        default: break;
                    }

                    ref readonly var end = ref wayPointBufferSpan[navAgentPathSteer.TargetPointIndex];

                    var deltaLength = (navAgentCom.speed * TimeDeltaTime);
                    //var radiu = deltaLength + navAgentCom.stopDistance + navAgentPathSteer.PickWayPointDistance;


                    var dir_normal = math.normalizesafe(wayPointBufferSpan[navAgentPathSteer.TargetPointIndex].Point - navAgentTransform.position, math.rotate(navAgentTransform.rotation, math.forward()));
                    var delta = dir_normal * deltaLength;
                    var deltaRadiu = deltaLength + navAgentCom.stopDistance;
                    //corro count
                    for (int i = navAgentPathSteer.TargetPointIndex; i < wayPointBufferSpan.Length - 1; i++)
                    {
                        if (math.distancesq(wayPointBufferSpan[i].Point, navAgentTransform.position) <= deltaRadiu * deltaRadiu)
                        {
                            navAgentPathSteer.start = navAgentTransform.position;
                            navAgentPathSteer.TargetPointIndex = i + 1;
                            break;
                        }
                    }

                    //end StreerStateover 最后移动一次
                    if (navAgentPathSteer.TargetPointIndex == wayPointBufferSpan.Length - 1)
                    {

                        if (math.distancesq(wayPointBufferSpan[navAgentPathSteer.TargetPointIndex].Point, navAgentTransform.position) <= deltaRadiu * deltaRadiu)
                        {
                            navAgentPathSteer.streerState = StreerState.Over;
                            navAgentPathSteer.start = navAgentTransform.position;
                            navAgentPathSteer.TargetPointIndex = wayPointBufferSpan.Length;
                        }
                    }

                    locations[count] = navAgentLocation.NavMeshLocation;
                    IndexArray[count] = nextIndex;
                    TargetArray[count] = navAgentTransform.position + delta;
                    AreaMask[count] = NavAgentComponentArray[count].areaMask;
                    var targetRotation = quaternion.LookRotationSafe(dir_normal, new float3(0, 1, 0));
                    navAgentTransform.rotation = math.slerp(navAgentTransform.rotation, targetRotation, TimeDeltaTime * navAgentCom.angleSpeed);
                    count++;
                }

                //Debug.Log(count);
                navMeshQuery.MoveLocations(locations.Slice(count), TargetArray.Slice(count), AreaMask.Slice(count));

                for (int i = 0; i < count; i++)
                {
                    var index = IndexArray[i];
                    NavAgentTransformArray[index].position = TargetArray[i];
                }
            }
        }


    }


}