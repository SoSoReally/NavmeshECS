using System;
using Game.Math;
using Unity.Burst;
using Unity.Burst.CompilerServices;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Experimental.AI;
using static Game.Navmesh.FindPathSystem;

namespace Game.Navmesh
{

    //note 如果导航steerstate.faile失败是否需要重新寻路？
    public static class PathSetting
    {
        private static int process = -1;
        public static int Process { get { if (process == -1) { process = Environment.ProcessorCount - 1; } return process; } } //处理器数量(-1)
        public const int MaxPathSize = 1000;           //可以获得最大路径
        public const int PreQuerySlot = 64;             //每一个QueryECS处理的QuerySlot
        public const int PreMaxIterator = 64;          //每一个QueryECS 每一帧处理的最大 路径迭代次数
        public const int InvaildQuerySlot = -1;
        //[BurstCompile]
        //public static Vector3 GetClosestPointOnFiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
        //{
        //    Vector3 line_direction = line_end - line_start;
        //    float line_length = line_direction.magnitude;
        //    line_direction.Normalize();
        //    float project_length = Mathf.Clamp(Vector3.Dot(point - line_start, line_direction), 0f, line_length);
        //    return line_start + line_direction * project_length;
        //}
        ////[BurstCompile]
        //// For infinite lines:
        //public static Vector3 GetClosestPointOnInfiniteLine(Vector3 point, Vector3 line_start, Vector3 line_end)
        //{
        //    return line_start + Vector3.Project(point - line_start, line_end - line_start);
        //}


        //[BurstCompile]
        //public static bool IsPointClosestLine(in float3 start, in float3 end, in float3 point, float radiu)
        //{
        //    if (math.all((start - end) == float3.zero)) return math.distancesq(start, point) <= radiu * radiu;
        //    var cloasetPoint = GetClosestPointOnFiniteLine(in start, in end, in point);
        //    return math.distancesq(cloasetPoint, point) <= radiu * radiu;
        //}
        ////[BurstCompile]
        //public static bool IsPointClosestLineSq(in float3 start, in float3 end, in float3 point, float radiusq)
        //{
        //    var cloasetPoint = GetClosestPointOnFiniteLine(in start, in end, in point);
        //    return math.distancesq(cloasetPoint, point) <= radiusq;
        //}

        ////[BurstCompile]
        //public static void GetClosestPointOnFiniteLine(in float3 start, in float3 end, in float3 point,ref float3 result)
        //{
        //    var u = point - start;
        //    var v = end - start;
        //    var length = math.length(v);
        //    var dir = math.normalize(v);
        //    var p_length = math.clamp(math.dot(u, dir), 0f, length);
        //    result = start + p_length * dir;
        //}
        ////[BurstCompile]
        //public static float3 GetClosestPointOnInfiniteLine(in float3 start, in float3 end, in float3 point)
        //{
        //    var u = point - start;
        //    var v = end - start;
        //    return start + math.project(u, math.normalize(v));
        //    //return  math.clamp(p+start,start,end);
        //}

        ////[BurstCompile]
        //public static float3 MoveTowards(float3 current, float3 target, float maxDistanceDelta)
        //{
        //    float3 lengthDir = target - current;
        //    var lengthsq = math.lengthsq(lengthDir);
        //    if (lengthsq == 0f || (maxDistanceDelta >= 0f && lengthsq <= maxDistanceDelta * maxDistanceDelta))
        //    {
        //        return target;
        //    }
        //    var length = math.sqrt(lengthsq);
        //    return current +  lengthDir / length * maxDistanceDelta;
        //}
    }

    public struct QueryECS : IDisposable
    {
        public NavMeshQuery NavMeshQuery;
        public byte UID;

        public IntPtr SlotPtr;
        public IntPtr OutputSlotPrt;
        //Global id
        public UnsafeRingQueue<int> freeSlot;
        //Global id
        public UnsafeRingQueue<int> useSolt;

        public UnsafeRingQueue<int> completeSlot;
        public void Init(byte uID, NativeArray<QuerySlot> querySlots, NativeArray<QuerySlotOutput> querySlotOutputs)
        {
            UID = uID;
            int ecsIndex = uID;
            var startIndex = ecsIndex* PathSetting.PreQuerySlot;
            var endIndex = startIndex + PathSetting.PreQuerySlot;
            //QuerySlots = querySlots;
            SlotPtr = querySlots.GetBufferUnSafeIntPtr();
            var span = SlotPtr.AsSpan<QuerySlot>(PathSetting.PreQuerySlot * PathSetting.Process);

            OutputSlotPrt = querySlotOutputs.GetBufferUnSafeIntPtr();
            NavMeshQuery = new NavMeshQuery(NavMeshWorld.GetDefaultWorld(), Allocator.Persistent, ushort.MaxValue);
            freeSlot = new UnsafeRingQueue<int>(PathSetting.PreQuerySlot, Allocator.Persistent);
            useSolt = new UnsafeRingQueue<int>(PathSetting.PreQuerySlot, Allocator.Persistent);
            completeSlot = new UnsafeRingQueue<int>(PathSetting.PreQuerySlot, Allocator.Persistent);

            for (int i = startIndex; i < endIndex; i++)
            {
                span[i].QueryECSIndex = uID;
                freeSlot.Enqueue(i);
            }
        }
        public void Update(ref QueryECSJob.TypeHandle handle)
        {
            Span<QuerySlot> spanSlot = SlotPtr.AsSpan<QuerySlot>(handle.AllSolt);
            var spanSlotOutput = OutputSlotPrt.AsSpan<QuerySlotOutput>(handle.AllSolt);
            var maxIterator = PathSetting.PreMaxIterator;
            //Debug.Log("slot"+useSolt.Length);
            while (useSolt.Length>0 && maxIterator>0)
            {
                var soltGlobalIndex =  useSolt.Dequeue();

                completeSlot.Enqueue(soltGlobalIndex);
                ref var slot = ref spanSlot[soltGlobalIndex];
                ref readonly var navgentTransform = ref handle.navgentTransformLookup.GetRefRO(slot.entity).ValueRO;
                ref readonly var navgentComponent = ref handle.navagetnComponentLookup.GetRefRO(slot.entity).ValueRO;
                ref readonly var request = ref handle.RequestLookup.GetRefRO(slot.entity).ValueRO;

                ref readonly var start = ref navgentTransform.position;
                ref readonly var end = ref request.position;
                NavMeshLocation startLocation = NavMeshQuery.MapLocation(start, navgentComponent.exetern, navgentComponent.agentType, navgentComponent.areaMask);
                NavMeshLocation endLocation = NavMeshQuery.MapLocation(end, navgentComponent.exetern, navgentComponent.agentType, navgentComponent.areaMask);
                if (!NavMeshQuery.IsValid(startLocation) || !NavMeshQuery.IsValid(endLocation) || math.distancesq(startLocation.position, endLocation.position) < navgentComponent.stopDistance * navgentComponent.stopDistance)
                {
                    slot.PathInfo.status = PathQueryStatus.Failure;
                    continue;
                }

                var queryState = NavMeshQuery.BeginFindPath(startLocation, endLocation, navgentComponent.areaMask);
                if (queryState.IsFailure())
                {
                    slot.PathInfo.status = PathQueryStatus.Failure;
                    continue;
                }

                //update stage
                while (true)
                {
                    var state = NavMeshQuery.UpdateFindPath(PathSetting.PreMaxIterator, out var iterationsPerformed);
                    maxIterator -= iterationsPerformed;
                    if (state.IsSuccess())
                    {
                        var endstate = NavMeshQuery.EndFindPath(out int pathSize);
                        if (pathSize > PathSetting.MaxPathSize)
                        {
                            slot.PathInfo.status = PathQueryStatus.Failure;
                        }
                        else
                        {
                            ref var output = ref spanSlotOutput[slot.Index];
                            if (endstate.IsSuccess())
                            {
                                NavMeshQuery.GetPathResult(output.polygonIds);
                                var pathStatus = PathUtils.FindStraightPath(NavMeshQuery, start, end, output.polygonIds, pathSize, ref output.straightPath, ref output.straightPathFlags, ref output.vertexSide, ref slot.PathInfo.cornerCount, PathSetting.MaxPathSize);
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
                                }
                                else
                                {
                                    slot.PathInfo.status = PathQueryStatus.Failure;
                                }

                            }
                            else
                            {

                                slot.PathInfo.status = PathQueryStatus.Failure;
                            }
                        }
                        break;
                    }
                    else if (state.IsFailure())
                    {
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
            freeSlot.Dispose();
            useSolt.Dispose();
            completeSlot.Dispose();
        }
    }
    public struct QuerySlot 
    {
        public Entity entity;
        //public RequestInfo Reuquest;
        public int QueryECSIndex;
        public int Index;
        //OutPut
        public PathInfo PathInfo;
        public static void ClearState(ref QuerySlot querySlot)
        {
            querySlot.PathInfo = new PathInfo() { cornerCount = 0, pathFoundToPosition = float3.zero, pathSize = 0,  status = PathQueryStatus.Failure };
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
        public int cornerCount;
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

    public enum RequestStateCode
    {
        Idle,
        InQueue,
        InPrograss,//考虑分帧请求
    }

    public struct RequestState : IComponentData,ICleanupComponentData
    {
        public RequestStateCode code;
        public int QuerySlotIndex;
    }
    public struct RequestInfo
    {
        public float3 start;
        public float3 end;
        public int mask;
        public float ThresholdDistance;
        public int agentType;
    }

    /// <summary>
    /// 路径请求
    /// </summary>
    public struct Request : IComponentData, IEnableableComponent
    {
        public float3 position;
        public RequestCode command;
        public enum RequestCode
        {
            Query,
            Cancel
        }
    }

    /// <summary>
    /// 路径请求的信息
    /// </summary>
    public struct PathInfo : IComponentData
    {
        public PathQueryStatus status;
        public int pathSize;
        public int cornerCount;
        public float3 pathFoundToPosition;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="pos"></param>
        /// <param name="threshold">lengthsq</param>
        /// <returns></returns>
        public readonly bool HasPathByTarget2d(in float2 pos ,float threshold)
        {
           return pathSize>0&& math.distancesq(pathFoundToPosition.xz,pos)<threshold&&!status.IsFailure();
        }

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
        public half speedSize;
        public half angleSpeedSize;
        public float stopDistance;
        public float3 exetern;
        public int areaMask;
        public int agentType;
        public float externLength;

        public readonly static NavAgentComponent Default = new NavAgentComponent() { angleSpeed = 360f, speed = 2f, stopDistance = 0.1f, agentType = 0, areaMask = -1, exetern = 1f , angleSpeedSize = new half(1f), speedSize = new half(1f)};
        
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


    public struct UnsafeParallelHashMapCount
    {
        public void CountAdd<TKey>(ref UnsafeParallelHashMap<TKey,int> map,in TKey value) where TKey : unmanaged , IEquatable<TKey>
        {
            if (map.ContainsKey(value))
            {
                map[value]++;
            }
            else
            {
                map.Add(value, 0);
            }
        }

        public bool CountRemove<TKey>(ref UnsafeParallelHashMap<TKey, int> map,in TKey value) where TKey : unmanaged, IEquatable<TKey>
        {
            if (map.ContainsKey(value))
            {
                map[value]--;
                if (map[value] <=0)
                {
                    map.Remove(value);
                }
                return true;
            }
            return false;
        }
    }
    public struct UnsafeHashMapCount
    {
        public void CountAdd<TKey>(ref UnsafeHashMap<TKey, int> map, in TKey value) where TKey : unmanaged, IEquatable<TKey>
        {
            if (map.ContainsKey(value))
            {
                map[value]++;
            }
            else
            {
                map.Add(value, 0);
            }
        }

        public bool CountRemove<TKey>(ref UnsafeHashMap<TKey, int> map, in TKey value) where TKey : unmanaged, IEquatable<TKey>
        {
            if (map.ContainsKey(value))
            {
                map[value]--;
                if (map[value] <= 0)
                {
                    map.Remove(value);
                }
                return true;
            }
            return false;
        }
    }

    public partial class NavmeshSystem : ComponentSystemGroup {

        public static SharedStatic<bool> DebugToggle;
    
    }

    public static class NavmeshUtility
    {
        public static void Setup(NativeArray<Entity> e, EntityManager entityManager, in NavAgentComponent navAgentComponent)
        {
            FixedList128Bytes<ComponentType> comList = new FixedList128Bytes<ComponentType>
            {
                Length = 8
            };
            comList.Clear();
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentComponent>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentTransform>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentPathSteer>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentLocation>());
            comList.AddNoResize(ComponentType.ReadWrite<Request>());
            comList.AddNoResize(ComponentType.ReadWrite<RequestState>());
            comList.AddNoResize(ComponentType.ReadWrite<PathInfo>());
            comList.AddNoResize(ComponentType.ReadWrite<WayPoint>());
            entityManager.AddComponent(e, new ComponentTypeSet(comList));
            for (int i = 0; i < e.Length; i++)
            {
                entityManager.SetComponentData<NavAgentComponent>(e[i],navAgentComponent );
                entityManager.SetComponentData<NavAgentTransform>(e[i], new NavAgentTransform() { position = Unity.Mathematics.float3.zero, rotation = quaternion.identity });
                entityManager.SetComponentData<NavAgentPathSteer>(e[i], new NavAgentPathSteer() { TargetPointIndex = 0, PickWayPointDistance = 0.2f });
                entityManager.SetComponentData<NavAgentLocation>(e[i], new NavAgentLocation() { });
                entityManager.SetComponentData<RequestState>(e[i], new RequestState() { code = RequestStateCode.Idle,QuerySlotIndex= PathSetting.InvaildQuerySlot });
                entityManager.SetComponentEnabled<Request>(e[i], false);
                entityManager.SetComponentEnabled<NavAgentComponent>(e[i], false);
            }

        }

        public static void Setup(NativeArray<Entity> e, EntityManager entityManager, Span<NavAgentComponent> navAgentComponent)
        {
            FixedList128Bytes<ComponentType> comList = new FixedList128Bytes<ComponentType>
            {
                Length = 8
            };
            comList.Clear();
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentComponent>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentTransform>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentPathSteer>());
            comList.AddNoResize(ComponentType.ReadWrite<NavAgentLocation>());
            comList.AddNoResize(ComponentType.ReadWrite<Request>());
            comList.AddNoResize(ComponentType.ReadWrite<RequestState>());
            comList.AddNoResize(ComponentType.ReadWrite<PathInfo>());
            comList.AddNoResize(ComponentType.ReadWrite<WayPoint>());
            entityManager.AddComponent(e, new ComponentTypeSet(comList));
            for (int i = 0; i < e.Length; i++)
            {
                entityManager.SetComponentData<NavAgentComponent>(e[i], navAgentComponent[i]);
                entityManager.SetComponentData<NavAgentTransform>(e[i], new NavAgentTransform() { position = Unity.Mathematics.float3.zero, rotation = quaternion.identity });
                entityManager.SetComponentData<NavAgentPathSteer>(e[i], new NavAgentPathSteer() { TargetPointIndex = 0, PickWayPointDistance = 0.2f });
                entityManager.SetComponentData<NavAgentLocation>(e[i], new NavAgentLocation() { });
                entityManager.SetComponentData<RequestState>(e[i], new RequestState() { code = RequestStateCode.Idle, QuerySlotIndex = PathSetting.InvaildQuerySlot });
                entityManager.SetComponentEnabled<Request>(e[i], false);
                entityManager.SetComponentEnabled<NavAgentComponent>(e[i], false);
            }

        }


        public static void Remove(NativeArray<Entity> entities, EntityManager entityManager)
        {
            entityManager.RemoveComponent<NavAgentComponent>(entities);
            entityManager.RemoveComponent<NavAgentTransform>(entities);
            entityManager.RemoveComponent<NavAgentPathSteer>(entities);
            entityManager.RemoveComponent<NavAgentLocation>(entities);
            entityManager.RemoveComponent<RequestState>(entities);
            entityManager.RemoveComponent<Request>(entities);
            entityManager.RemoveComponent<PathInfo>(entities);
            entityManager.RemoveComponent<WayPoint>(entities);
        }
    }
    /// <summary>
    /// 这个系统负责寻找路径
    /// </summary>
    // 寻路请求会添加到寻路列表中
    // 寻路数据在寻路时才会确定
    // 寻路取消会在取消队列里面 会修改寻路状态为idle
    // 寻路取消后 重新请求会进入寻路列表中
    [UpdateInGroup(typeof(NavmeshSystem))]
    [BurstCompile]
    public partial struct FindPathSystem : ISystem
    {

        public struct Data: IComponentData,IDisposable
        {
            public UnsafeParallelHashMap<Entity, int> CancelRequest;
            public UnsafeHashSet<int> CancelSlot;
            public Data(int capcity)
            {
                CancelRequest = new UnsafeParallelHashMap<Entity, int>(64, Allocator.Persistent);
                CancelSlot = new UnsafeHashSet<int>(16, Allocator.Persistent);
            }

            public void Dispose()
            {
                CancelRequest.Dispose();
                CancelSlot.Dispose();
            }
        }

        public NativeArray<QueryECS> QueryECSArray;
        public NativeArray<QuerySlot> QuerySlotArray;
        public NativeArray<QuerySlotOutput> QuerySlotOutputArray;

        public EntityQuery RequestPathQuery_1;
        public EntityTypeHandle EntityTypeHandle;
        public UnsafeList<Entity> QueueRequestList;

        public CheckRequestAndEntryQueryQueue.TypeHandle EntryQueryQueueTypeHandle;

        public QueryECSJob.TypeHandle QueryECSJobHandle;

        public GetQuerySlotOutputJob.TypeHandle GetQuerySlotOutputJobTypeHandle;

        public CheckAndEntryQuerySlotJob.TypeHandle CheckAndEntryQuerySlotJobHandle;
        public EntryCancleQuery.TypeHandle EntryCancleQueryTypeHandle;
        public int allSlot;

        public EntityQuery ClearQuery;
        public ClearEntity.TypeHandle ClearEntityTypeHandle;
        [BurstCompile]
        public partial struct ClearEntity : IJobChunk
        {
            public EntityCommandBuffer ecb;
            [NativeDisableUnsafePtrRestriction]
            public IntPtr CancelList;
            public TypeHandle typeHandle;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                ref var data = ref CancelList.As<Data>();
                ref var cancel = ref data.CancelRequest;
                UnsafeParallelHashMapCount mapcount = default;
                var readonlySpanRequestState = chunk.GetNativeArray(ref typeHandle.RequestStateComponentTypeHandle).AsReadOnlySpan();
                var Entitys = chunk.GetNativeArray(typeHandle.EntityHandle).AsReadOnlySpan();
                //Debug.Log("why no " + Entitys.Length);
                //Debug.Log(chunk.Count);
                for (int i = 0; i < chunk.Count; i++)
                {
                    if (readonlySpanRequestState[i].code == RequestStateCode.InQueue)
                    {
                        mapcount.CountAdd(ref cancel, in Entitys[i]);
                    }
                    else if (readonlySpanRequestState[i].code == RequestStateCode.InPrograss)
                    {
                        data.CancelSlot.Add(readonlySpanRequestState[i].QuerySlotIndex);
                    }
                }


                ComponentTypeSet componentTypeSet = new ComponentTypeSet(ComponentType.ReadWrite<RequestState>());
                ecb.RemoveComponent(chunk.GetNativeArray(typeHandle.EntityHandle), in componentTypeSet);
            }
            public struct TypeHandle
            {
                [ReadOnly]
                public ComponentTypeHandle<RequestState> RequestStateComponentTypeHandle;
                [ReadOnly]
                public EntityTypeHandle EntityHandle;
                public TypeHandle(ref SystemState systemState)
                {
                    EntityHandle = systemState.GetEntityTypeHandle();
                    RequestStateComponentTypeHandle = systemState.GetComponentTypeHandle<RequestState>(true);
                }
                public void Update(ref SystemState systemState)
                {
                    EntityHandle.Update(ref systemState);
                    RequestStateComponentTypeHandle.Update(ref systemState);
                }
            }
        }
        void OnCreate(ref SystemState systemState)
        {
            allSlot = PathSetting.Process * PathSetting.PreQuerySlot;
            QueryECSArray = new NativeArray<QueryECS>(PathSetting.Process, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            QuerySlotArray = new NativeArray<QuerySlot>(allSlot, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            QuerySlotOutputArray = new NativeArray<QuerySlotOutput>(allSlot, Allocator.Persistent, NativeArrayOptions.UninitializedMemory);
            var spanQuerySlot = QuerySlotArray.AsSpan();
            var spanQuerySlotOutput = QuerySlotOutputArray.AsSpan();
            var QueryECSArraySpan = QueryECSArray.AsSpan();
            for (int i = 0; i < QuerySlotArray.Length; i++)
            {
                ref var value =ref spanQuerySlot[i];
                ref var output = ref spanQuerySlotOutput[i];
                value.Index = i;
                QuerySlot.ClearState(ref value);
                output = QuerySlotOutput.Default();
            }
            
            for (int i = 0; i < PathSetting.Process; i++)
            {
                //var start = i * PathSetting.PreQuerySlot;
                //var length = PathSetting.PreQuerySlot;
                QueryECSArraySpan[i].Init((byte)i, QuerySlotArray, QuerySlotOutputArray);
            }

            systemState.WorldUnmanaged.EntityManager.AddComponentData<Data>(systemState.SystemHandle, new Data(16) { });
            QueueRequestList = new UnsafeList<Entity>(allSlot, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            RequestPathQuery_1 = systemState.GetEntityQuery(typeof(Request), typeof(RequestState)) ;
            
            EntryQueryQueueTypeHandle = new CheckRequestAndEntryQueryQueue.TypeHandle(ref systemState);
            QueryECSJobHandle.Init(ref systemState);
            GetQuerySlotOutputJobTypeHandle = new GetQuerySlotOutputJob.TypeHandle(ref systemState);
            CheckAndEntryQuerySlotJobHandle = new CheckAndEntryQuerySlotJob.TypeHandle(ref systemState);
            EntryCancleQueryTypeHandle = new EntryCancleQuery.TypeHandle(ref systemState);

            ClearQuery = systemState.GetEntityQuery(new EntityQueryBuilder(Allocator.Temp).WithNone<WayPoint>().WithNone<NavAgentComponent>().WithAll<RequestState>().WithOptions(EntityQueryOptions.IncludeDisabledEntities));
            ClearEntityTypeHandle = new ClearEntity.TypeHandle(ref systemState);
        }

        void OnUpdate(ref SystemState systemState)
        {
            EntityTypeHandle.Update(ref systemState);
            EntryQueryQueueTypeHandle.Update(ref systemState);
            QueryECSJobHandle.Update(ref systemState);
            GetQuerySlotOutputJobTypeHandle.Update(ref systemState);
            CheckAndEntryQuerySlotJobHandle.Update(ref systemState);
            EntryCancleQueryTypeHandle.Update(ref systemState);
            var maxEntryEntity = RequestPathQuery_1.CalculateEntityCountWithoutFiltering();
            if (QueueRequestList.Capacity - QueueRequestList.Length < maxEntryEntity)
            {
                QueueRequestList.SetCapacity(QueueRequestList.Capacity + maxEntryEntity);
            }

            //return;
            if (!ClearQuery.IsEmptyIgnoreFilter)
            {
                ClearEntityTypeHandle.Update(ref systemState);
                systemState.Dependency = new ClearEntity()
                {
                    ecb = SystemAPI.GetSingleton<EndSimulationEntityCommandBufferSystem.Singleton>().CreateCommandBuffer(systemState.WorldUnmanaged),
                    typeHandle = ClearEntityTypeHandle,
                    CancelList = SystemAPI.GetSingletonRW<Data>().ValueRW.AsIntPtr(),
                }.Schedule(ClearQuery, systemState.Dependency);

                //var d = SystemAPI.GetSingletonRW<Data>().ValueRW.CancelRequest;
                //var cs = SystemAPI.GetSingletonRW<Data>().ValueRW.CancelSlot;
                //Debug.Log(d.Count());
                //Debug.Log(cs.Count);
                //foreach (var item in d)
                //{
                //    Debug.Log(item.Key);
                //}
                //foreach (var item in cs)
                //{
                //    Debug.Log(item);
                //}
                //EntitiesJournaling.Enabled = false;
            }



            systemState.Dependency = new EntryCancleQuery()
            {
                CancelData = SystemAPI.GetComponentRW<Data>(systemState.SystemHandle).ValueRW.AsIntPtr(),
                typeHandle = EntryCancleQueryTypeHandle
            }.Schedule(RequestPathQuery_1, systemState.Dependency);

            systemState.Dependency = new CheckRequestAndEntryQueryQueue()
            {
                EntityHandle = EntityTypeHandle,
                QueueListWriter = QueueRequestList.AsParallelWriter(),
                typeHandle = EntryQueryQueueTypeHandle
            }.ScheduleParallel(RequestPathQuery_1, systemState.Dependency);

            if (QueueRequestList.Length>0)
            {
                systemState.Dependency = new CheckAndEntryQuerySlotJob()
                {
                    typeHandle = CheckAndEntryQuerySlotJobHandle,
                    allSlot = allSlot,
                    process = PathSetting.Process,
                    QueryListPtr = QueueRequestList.AsIntPtr(),
                    querySlots = QuerySlotArray.GetBufferUnSafeIntPtr(),
                    QueryECSArray = QueryECSArray.GetBufferUnSafeIntPtr(),
                    CancelData = SystemAPI.GetComponentRW<Data>(systemState.SystemHandle).ValueRW.AsIntPtr()
                }.Schedule(systemState.Dependency);
            }


            systemState.Dependency = new QueryECSJob()
            {
                handle = QueryECSJobHandle,
                QueryECSPtr = QueryECSArray.GetBufferUnSafeIntPtr(),
                Process = PathSetting.Process,
            }.Schedule(PathSetting.Process, 1, systemState.Dependency);

            systemState.Dependency = new GetQuerySlotOutputJob()
            {
                AllSlot = allSlot,
                process = PathSetting.Process,
                QueryECS = QueryECSArray.GetBufferUnSafeIntPtr(),
                typeHandle = GetQuerySlotOutputJobTypeHandle,
                QuerySlotsArray = QuerySlotArray.GetBufferUnSafeIntPtr(),
                QuerySlotsOutputArray = QuerySlotOutputArray.GetBufferUnSafeIntPtr()
            }.Schedule(PathSetting.Process, 1, systemState.Dependency);


        }
        void OnDestroy(ref SystemState systemState)
        {
            systemState.CompleteDependency();
            for (int i = 0; i < QuerySlotOutputArray.Length; i++)
            {
                QuerySlotOutputArray[i].Dispose();
            }
            QuerySlotArray.Dispose();
            QueryECSArray.Dispose();
            QuerySlotOutputArray.Dispose();

            QueueRequestList.Dispose();
            SystemAPI.GetComponentRW<Data>(systemState.SystemHandle).ValueRW.Dispose();
        }


        /// <summary>
        /// when delete entity
        /// </summary>

        /// <summary>
        /// check and join queue
        /// </summary>
        [BurstCompile]
        public partial struct CheckRequestAndEntryQueryQueue : IJobChunk
        {
            public TypeHandle typeHandle;
            public UnsafeList<Entity>.ParallelWriter QueueListWriter;
            [ReadOnly]
            public EntityTypeHandle EntityHandle;
            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var mask = chunk.GetEnabledMask(ref typeHandle.RequestCommandHandle);
                var Enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                var readOnlyuSpanRequestCommand = chunk.GetNativeArray(ref typeHandle.RequestCommandHandle).AsReadOnlySpan();
                var spanRequestState = chunk.GetNativeArray(ref typeHandle.RequesStateHanlde).AsSpan();
                var readOnlyEntity = chunk.GetNativeArray(EntityHandle).AsReadOnlySpan();
                while (Enumerator.NextEntityIndex(out var index))
                {
                    mask[index] = false;
                    if (readOnlyuSpanRequestCommand[index].command == Request.RequestCode.Query)
                    {
                        if (spanRequestState[index].code == RequestStateCode.Idle)
                        {
                            QueueListWriter.AddNoResize(readOnlyEntity[index]);
                            spanRequestState[index].code = RequestStateCode.InQueue;
                        }
                    }
                }
            }


            public struct TypeHandle
            {
                public ComponentTypeHandle<Request> RequestCommandHandle;
                public ComponentTypeHandle<RequestState> RequesStateHanlde;
                public TypeHandle(ref SystemState systemState)
                {
                    RequestCommandHandle = systemState.GetComponentTypeHandle<Request>(false);
                    RequesStateHanlde = systemState.GetComponentTypeHandle<RequestState>(false);
                }
                public void Update(ref SystemState systemState)
                {
                    RequestCommandHandle.Update(ref systemState);
                    RequesStateHanlde.Update(ref systemState);

                }
            }

        }
        [BurstCompile]
        public partial struct EntryCancleQuery : IJobChunk
        {
            [NativeDisableUnsafePtrRestriction]
            public IntPtr CancelData;
            public TypeHandle typeHandle;
            public struct TypeHandle
            {
                [ReadOnly]
                public ComponentTypeHandle<Request> RequestCommandHandle;

                public ComponentTypeHandle<RequestState> RequesStateHanlde;
                [ReadOnly]
                public EntityTypeHandle EntityHandle;
                public TypeHandle(ref SystemState systemState)
                {
                    RequestCommandHandle = systemState.GetComponentTypeHandle<Request>(true);
                    RequesStateHanlde = systemState.GetComponentTypeHandle<RequestState>(false);
                    EntityHandle = systemState.GetEntityTypeHandle();
                }
                public void Update(ref SystemState systemState)
                {
                    RequestCommandHandle.Update(ref systemState);
                    RequesStateHanlde.Update(ref systemState);
                    EntityHandle.Update(ref systemState);

                }
            }

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var Enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
                var readOnlyuSpanRequestCommand = chunk.GetNativeArray(ref typeHandle.RequestCommandHandle).AsReadOnlySpan();
                var spanRequestState = chunk.GetNativeArray(ref typeHandle.RequesStateHanlde).AsSpan();
                var readOnlyEntity = chunk.GetNativeArray(typeHandle.EntityHandle).AsReadOnlySpan();
                ref var data = ref CancelData.As<Data>();
                ref var map = ref data.CancelRequest;
                UnsafeParallelHashMapCount unsafeParallelHashMapCount = default;
                while (Enumerator.NextEntityIndex(out var index))
                {
                    if (readOnlyuSpanRequestCommand[index].command == Request.RequestCode.Cancel)
                    {
                        if (spanRequestState[index].code == RequestStateCode.InQueue)
                        {
                            spanRequestState[index].QuerySlotIndex = PathSetting.InvaildQuerySlot;
                            spanRequestState[index].code = RequestStateCode.Idle;
                            unsafeParallelHashMapCount.CountAdd(ref map, in readOnlyEntity[index]);
                        }
                        else if(spanRequestState[index].code == RequestStateCode.InPrograss)
                        {
                            data.CancelSlot.Add(spanRequestState[index].QuerySlotIndex);
                            spanRequestState[index].QuerySlotIndex = PathSetting.InvaildQuerySlot;
                            spanRequestState[index].code = RequestStateCode.Idle;
                        }
                    }
                }

            }
        }

        /// <summary>
        /// check form queue and join slot
        /// </summary>
        [BurstCompile()]
        public partial struct CheckAndEntryQuerySlotJob :IJob
        {
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryListPtr;

            [NativeDisableUnsafePtrRestriction]
            public IntPtr CancelData;
            [ReadOnly]
            public int allSlot;
            [ReadOnly]
            public int process;

            [NativeDisableUnsafePtrRestriction()]
            public IntPtr querySlots;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECSArray;

            public TypeHandle typeHandle;

            [SkipLocalsInit()]
            public void Execute()
            {
                ref var data = ref CancelData.As<Data>();
                ref var cancelRequest = ref data.CancelRequest;
                var spanQuerySlot = querySlots.AsSpan<QuerySlot>(allSlot);
                var que = QueryECSArray.AsSpan<QueryECS>(process);
                var cancelOnSlot = data.CancelSlot.Count;

                if (cancelOnSlot>0)
                {
                    for (int i = 0; i < que.Length; i++)
                    {
                        var tempLength = que[i].useSolt.Length;
                        for (int ri = tempLength-1; ri >= 0; ri--)
                        {
                            var dvalue = que[i].useSolt.Dequeue();
                            if (data.CancelSlot.Contains(dvalue))
                            {
                                //Debug.Log(dvalue);
                                data.CancelSlot.Remove(dvalue);
                                que[i].freeSlot.Enqueue(dvalue);
                            }
                            else
                            {
                                que[i].useSolt.Enqueue(dvalue);
                            }
                        }
                    }
                }

                if (data.CancelSlot.Count>0)
                {
                    //foreach (var item in data.CancelSlot)
                    //{
                    //    Debug.Log(item);
                    //}
                    //Debug.Log("取消的slot 不在useslot中");
                    return;
                }

                UnsafeParallelHashMapCount unsafeParallelHashMapCount = default;
                ref var QueryList = ref QueryListPtr.As<UnsafeList<Entity>>();
                var allEntity = QueryList.Length;
                if (allEntity <= 0)
                {
                    return;
                }



                var idleSlot = 0;
                for (int i = 0; i < que.Length; i++)
                {
                    idleSlot += que[i].freeSlot.Length;
                }
                //Debug.Log(idleSlot);
                //没有槽位
                if (idleSlot <= 0)
                {
                    return;
                }
                var min = math.min(allEntity, idleSlot);
                var ecsIndex = 0;
                var invalidEntityoffest = 0;
                var actualRemove = -1;
                 
               
                for (int i = 0; i < min; i++)
                {
                    actualRemove = i + invalidEntityoffest;
                    if (actualRemove >= QueryList.Length)
                    {
                        //actualRemove = [0,list.length)
                        actualRemove--;
                        break;
                    }
                    if (unsafeParallelHashMapCount.CountRemove(ref cancelRequest, in QueryList.ElementAt(actualRemove)))
                    {
                        //Debug.Log(QueryList.ElementAt(actualRemove) +" sss");
                        invalidEntityoffest++;
                        i--;                        
                        continue;
                    }

                    if (que[ecsIndex].freeSlot.Length<=0)
                    {
                        ecsIndex++;
                        i--;
                        continue;
                    }
                    var soltIndex = que[ecsIndex].freeSlot.Dequeue();
                    que[ecsIndex].useSolt.Enqueue(soltIndex);
                    //Debug.Log(que[ecsIndex].useSolt.Length +"|"+soltIndex);
                    QuerySlot.ClearState(ref spanQuerySlot[soltIndex]);
                    
                    spanQuerySlot[soltIndex].entity = QueryList.ElementAt(actualRemove);
                    ref var rs = ref typeHandle.RequestStateLookup.AsRef(in spanQuerySlot[soltIndex].entity);
                    rs.QuerySlotIndex = soltIndex;
                    rs.code = RequestStateCode.InPrograss;
                }
                //actualRemove is removeIndex so plus one
                QueryList.RemoveRange(0, actualRemove+1);
            }

            public struct TypeHandle
            {
                [NativeDisableParallelForRestriction]
                [NativeDisableContainerSafetyRestriction]
                public ComponentLookup<RequestState> RequestStateLookup;

                public ComponentLookup<Request> RequestLookup;
                public TypeHandle(ref SystemState systemState)
                {
                    RequestStateLookup = systemState.GetComponentLookup<RequestState>();
                    RequestLookup = systemState.GetComponentLookup<Request>();
                }
                public void Update(ref SystemState systemState)
                {
                    RequestStateLookup.Update(ref  systemState);
                    RequestLookup.Update(ref systemState);
                }
            }
        }
        /// <summary>
        /// findPath job
        /// </summary>
        [BurstCompile]
        public struct QueryECSJob : IJobParallelFor
        {
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECSPtr;
            public TypeHandle handle;
            [ReadOnly]
            public int Process;


            public void Execute(int index)
            {
                QueryECSPtr.AsSpan<QueryECS>(Process)[index].Update(
                    ref handle);
            }
            public struct TypeHandle
            {
                [ReadOnly]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<NavAgentTransform> navgentTransformLookup;
                [ReadOnly]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<NavAgentComponent> navagetnComponentLookup;
                [ReadOnly]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<Request> RequestLookup;

                [ReadOnly]
                public int AllSolt;
                public void Init(ref SystemState systemState)
                {
                    AllSolt = PathSetting.Process * PathSetting.PreQuerySlot;
                    navgentTransformLookup = systemState.GetComponentLookup<NavAgentTransform>(true);
                    navagetnComponentLookup = systemState.GetComponentLookup<NavAgentComponent>(true);
                    RequestLookup = systemState.GetComponentLookup<Request>(true);
                    //RequestStateLookup = systemState.GetComponentLookup<RequestState>(false);
                }
                public void Update(ref SystemState systemState)
                {
                    navgentTransformLookup.Update(ref systemState);
                    navagetnComponentLookup.Update(ref systemState);
                    RequestLookup.Update(ref systemState);
                    //RequestStateLookup.Update(ref systemState);
                }
            }
        }
        /// <summary>
        /// copy path
        /// </summary>
        [BurstCompile]
        public struct GetQuerySlotOutputJob : IJobParallelFor
        {
            public TypeHandle typeHandle;
            [ReadOnly]
            public int AllSlot;
            [ReadOnly]
            public int process;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QuerySlotsArray;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QuerySlotsOutputArray;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECS;
            //public void Execute()
            //{

            //}

            public struct TypeHandle
            {
                [NativeDisableContainerSafetyRestriction]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<PathInfo> PathInfoLookup;
                [NativeDisableContainerSafetyRestriction]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<NavAgentPathSteer> NavAgentPathSteerLookup;
                [NativeDisableContainerSafetyRestriction]
                [NativeDisableParallelForRestriction]
                public ComponentLookup<RequestState> RequestStateLookup;


                [NativeDisableContainerSafetyRestriction]
                [NativeDisableParallelForRestriction]
                public BufferLookup<WayPoint> WayPointLookup;

                public TypeHandle(ref SystemState systemState)
                {
                    PathInfoLookup = systemState.GetComponentLookup<PathInfo>();
                    NavAgentPathSteerLookup = systemState.GetComponentLookup<NavAgentPathSteer>();
                    WayPointLookup = systemState.GetBufferLookup<WayPoint>();
                    RequestStateLookup= systemState.GetComponentLookup<RequestState>();
                }

                public void Update(ref SystemState systemState)
                {
                    PathInfoLookup.Update(ref systemState);
                    NavAgentPathSteerLookup.Update(ref systemState);
                    WayPointLookup.Update(ref systemState);
                    RequestStateLookup.Update(ref systemState);
                }
            }
            public void Execute(int index)
            {
                var spanSlotOutput = QuerySlotsOutputArray.AsReadOnlySpan<QuerySlotOutput>(AllSlot);
                var spanSlot = QuerySlotsArray.AsSpan<QuerySlot>(AllSlot);
                var spanQueryECS = QueryECS.AsSpan<QueryECS>(process);
                ref var queryECS =ref spanQueryECS[index];
                //Debug.Log(queryECS.completeSlot.Length + "xx");
                //Debug.Log(queryECS.freeSlot.Length + "free");
                //Debug.Log(queryECS.useSolt.Length + "useSlot");
                while (queryECS.completeSlot.Length>0)
                {
                    int i = queryECS.completeSlot.Dequeue();
                    queryECS.freeSlot.Enqueue(i);
                    //Debug.Log("comp" + i);
                    ref var item = ref spanSlot[i];
                    ref var requestState = ref typeHandle.RequestStateLookup.AsRef(in item.entity);
                    requestState.code = RequestStateCode.Idle;
                    requestState.QuerySlotIndex = PathSetting.InvaildQuerySlot;
                    switch (item.PathInfo.status)
                    {
                        case PathQueryStatus.Success:

                            var wayPointDynamic = typeHandle.WayPointLookup[item.entity];
                            wayPointDynamic.Clear();

                            var PathInfo = typeHandle.PathInfoLookup.GetRefRW(item.entity);
                            PathInfo.ValueRW = item.PathInfo;
                            //ecb.SetComponent<PathInfo>(item.Entity, item.PathInfo);
                            //wayPointDynamic.AddRange(spanSlotOutput[i].wayPoints.AsArray());
                            //Debug.Log(spanSlotOutput[i].wayPoints.Length + "query lengt1");
                            var waypointSpan = spanSlotOutput[i].wayPoints.AsReadOnlySpan();
                            var NavAgentPathSteerRW = typeHandle.NavAgentPathSteerLookup.GetRefRW(item.entity);
                            NavAgentPathSteer.Start(ref NavAgentPathSteerRW.ValueRW);
                            //Debug.Log(waypointSpan.Length + "query lengt");
                            wayPointDynamic.AddRange(waypointSpan);
                            //wayPointDynamic.AddRange(spanSlotOutput[i].wayPoints.AsArray())
                            QuerySlot.ClearState(ref item);
                            break;
                        case PathQueryStatus.Failure:
                            var wayPointDynamicFaile = typeHandle.WayPointLookup[item.entity];
                            wayPointDynamicFaile.Clear();
                            var PathInfoFail = typeHandle.PathInfoLookup.GetRefRW(item.entity);
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
    [RequireMatchingQueriesForUpdate]
    [BurstCompile]
    public partial struct NavmeshPathUpdateSystem : ISystem
    {
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
        public ComponentTypeHandle<NavAgentPathSteer> NavAgentPathSteerHandle2;
        public ComponentTypeHandle<NavAgentTransform> NavAgentTransformHandle2;
        public ComponentTypeHandle<NavAgentLocation> NavAgentLocationHandle2;


         void OnCreate(ref SystemState systemState)
        {
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
            CheckNavAgentComponentEnableEntityQuery_1 = systemState.GetEntityQuery(new EntityQueryBuilder(Allocator.Temp)
                .WithAll(ref querytemp)
                .WithOptions(EntityQueryOptions.IgnoreComponentEnabledState));


            NavAgentComponentHandle = systemState.GetComponentTypeHandle<NavAgentComponent>(false);
            NavAgentLocationHandle = systemState. GetComponentTypeHandle<NavAgentLocation>(false);
            NavTransformTypeHandle = systemState. GetComponentTypeHandle<NavAgentTransform>(true);
            // EntityTypeHandle = GetEntityTypeHandle();
            PathInfoHandle = systemState.GetComponentTypeHandle<PathInfo>(true);
            WayPointHandle = systemState.GetBufferTypeHandle<WayPoint>(true);
            //TransformHandle = new TransformAspect.TypeHandle(ref CheckedStateRef, true);
            NavAgentPathSteerHandle = systemState.GetComponentTypeHandle<NavAgentPathSteer>(true);


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
            SyncTransfromEntityQuery_2 = systemState.GetEntityQuery(new EntityQueryBuilder(Allocator.Temp).WithAll(ref querytemp));

            WayPointHandle2 = systemState.GetBufferTypeHandle<WayPoint>(true);
            NavAgentComponentHandle2 = systemState.GetComponentTypeHandle<NavAgentComponent>(true);
            //[ReadOnly] public ComponentTypeHandle<PathInfo> PathInfoHandle;
            NavAgentPathSteerHandle2 = systemState.GetComponentTypeHandle<NavAgentPathSteer>(false);
            NavAgentTransformHandle2 = systemState.GetComponentTypeHandle<NavAgentTransform>(false);
            NavAgentLocationHandle2 = systemState.GetComponentTypeHandle<NavAgentLocation>(false);
        }

        void OnUpdate(ref SystemState systemState)
        {
            NavAgentComponentHandle.Update(ref systemState);
            NavTransformTypeHandle.Update(ref systemState);
            //EntityTypeHandle.Update(this);
            PathInfoHandle.Update(ref systemState);
            WayPointHandle.Update(ref systemState);
            //TransformHandle.Update(ref CheckedStateRef);
            EnableNavAgentComponent.Update(ref systemState);
            NavAgentPathSteerHandle.Update(ref systemState);
            NavAgentLocationHandle.Update(ref systemState);
            var deltaTime = SystemAPI.Time.DeltaTime;
            //--------stage1
            systemState.Dependency = new CheckEnableNavAgentWithLocation()
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
            }.ScheduleParallel(CheckNavAgentComponentEnableEntityQuery_1, systemState. Dependency);

            //--------stage2
            //Dependency = new CheckPathAndSyncLocationJob() { navMeshQuery = OnlyLocation }.ScheduleParallel(Dependency);


            NavAgentComponentHandle2.Update(ref systemState);
            NavAgentLocationHandle2.Update(ref systemState);
            NavAgentPathSteerHandle2.Update(ref systemState);
            NavAgentTransformHandle2.Update(ref systemState);
            WayPointHandle2.Update(ref systemState);
            systemState.Dependency = new UpdateNavAgentTransformJob()
            {
                TimeDeltaTime = deltaTime,
                navMeshQuery = OnlyLocation,
                NavAgentComponentHandle = NavAgentComponentHandle2,
                NavAgentLocationHandle = NavAgentLocationHandle2,
                NavAgentPathSteerHandle = NavAgentPathSteerHandle2,
                NavAgentTransformHandle = NavAgentTransformHandle2,
                //TransformAspectTypeHandle = TransformAspectTypeHandle2,
                WayPointHandle = WayPointHandle2,
            }.ScheduleParallel(SyncTransfromEntityQuery_2, systemState.Dependency);

        }

        void OnDestroy(ref SystemState systemState)
        {
            OnlyLocation.Dispose();
        }

        public static ComponentType[] RequiredComponents => new ComponentType[]{
        typeof(NavAgentComponent),
        typeof(NavAgentTransform),
        typeof(NavAgentPathSteer),
        typeof(NavAgentLocation),
        typeof(Request),
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
                    var stopdistance = itemNavAgentCom.stopDistance;
                    //1
                    if (
                        //itemPathInfo.status != PathQueryStatus.Success
                        //||
                        waypointLength <= 0
                        || itemNavPathIndex.streerState == StreerState.Faile
                        || itemNavPathIndex.streerState == StreerState.Over
                        || itemNavPathIndex.TargetPointIndex >= waypointLength
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
                        //Debug.Assert(isShouldEnable, "location fail");
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
                        case StreerState.Over:continue;
                        default: break;
                    }
                    var deltaLength = (navAgentCom.speed * navAgentCom.speedSize* TimeDeltaTime);
                    //var deltaRadiu = deltaLength + navAgentCom.stopDistance;

                    float3 final = float3.zero;
                    float2 deltaDir2D = new float2(0,1);
                    float3 delatNoraml = math.mul(navAgentTransform.rotation, math.forward());
                    for (; navAgentPathSteer.TargetPointIndex < wayPointBufferSpan.Length; navAgentPathSteer.TargetPointIndex++)
                    {
                        ref readonly var end = ref wayPointBufferSpan[navAgentPathSteer.TargetPointIndex];
                        var deltaDir3D = (end.Point - navAgentTransform.position);
                        deltaDir2D = deltaDir3D.xz;
                        var lengthsq = math.lengthsq(deltaDir2D);

                        if (lengthsq == 0f || (deltaLength >= 0f && lengthsq <= deltaLength * deltaLength))
                        {
                            if (navAgentPathSteer.TargetPointIndex >= wayPointBufferSpan.Length-1)
                            {
                                final = wayPointBufferSpan[wayPointBufferSpan.Length - 1].Point;
                                navAgentPathSteer.streerState = StreerState.Over;
                            }
                            continue;
                        }
                        else
                        {
                            //2d length
                            var length = math.sqrt(lengthsq);

                            //xz 是normal y 是dot y-length
                            var dotDelat3d = deltaDir3D / length;
                            delatNoraml = dotDelat3d;
                            final = navAgentTransform.position + (dotDelat3d * deltaLength);

                            break;
                        }
                    }


                    //for (int i = navAgentPathSteer.TargetPointIndex; i < wayPointBufferSpan.Length - 1; i++)
                    //{
                    //    if (math.distancesq(wayPointBufferSpan[i].Point, navAgentTransform.position) <= deltaRadiu * deltaRadiu)
                    //    {
                    //        navAgentPathSteer.start = navAgentTransform.position;
                    //        navAgentPathSteer.TargetPointIndex = i + 1;
                    //        break;
                    //    }
                    //}

                    //var dir_normal = math.normalizesafe(wayPointBufferSpan[navAgentPathSteer.TargetPointIndex].Point - navAgentTransform.position, math.rotate(navAgentTransform.rotation, math.forward()));
                    //var delta = dir_normal * deltaLength;
                    ////corro count


                    ////end StreerStateover 最后移动一次
                    //if (navAgentPathSteer.TargetPointIndex == wayPointBufferSpan.Length - 1)
                    //{

                    //    if (math.distancesq(wayPointBufferSpan[navAgentPathSteer.TargetPointIndex].Point, navAgentTransform.position) <= deltaRadiu * deltaRadiu)
                    //    {
                    //        navAgentPathSteer.streerState = StreerState.Over;
                    //        navAgentPathSteer.start = navAgentTransform.position;
                    //        navAgentPathSteer.TargetPointIndex = wayPointBufferSpan.Length;
                    //    }
                    //}

                    locations[count] = navAgentLocation.NavMeshLocation;
                    IndexArray[count] = nextIndex;
                    TargetArray[count] = final;
                    AreaMask[count] = NavAgentComponentArray[count].areaMask;
                    delatNoraml.y = 0f;
                    var targetRotation = quaternion.LookRotation(delatNoraml, new float3(0, 1, 0));
                    navAgentTransform.rotation= mathEX.RotateTowards(in navAgentTransform.rotation, in targetRotation, navAgentCom.angleSpeed * navAgentCom.angleSpeedSize * TimeDeltaTime);
                    //navAgentTransform.rotation = math.slerp(navAgentTransform.rotation, targetRotation, TimeDeltaTime * navAgentCom.angleSpeed);
                    count++;
                }

                ////Debug.Log(count);
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