using System;
using Game.Math;
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
using static Doozy.Engine.Utils.ColorModels.RGB;
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

        [BurstCompile]
        public static float3 MoveTowards(float3 current, float3 target, float maxDistanceDelta)
        {
            float3 lengthDir = target - current;
            var lengthsq = math.lengthsq(lengthDir);
            if (lengthsq == 0f || (maxDistanceDelta >= 0f && lengthsq <= maxDistanceDelta * maxDistanceDelta))
            {
                return target;
            }
            var length = math.sqrt(lengthsq);
            return current +  lengthDir / length * maxDistanceDelta;
        }
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
        public void Update(ref QueryECSJob.TypeHandle handle, IntPtr cancelQueryPtr,in UnsafeList<Entity>.ParallelWriter definiteCancel)
        {
            ref var cancelQuery =ref cancelQueryPtr.As<UnsafeParallelHashSet<Entity>>();
            Span<QuerySlot> spanSlot = SlotPtr.AsSpan<QuerySlot>(handle.AllSolt);
            var spanSlotOutput = OutputSlotPrt.AsSpan<QuerySlotOutput>(handle.AllSolt);
            var maxIterator = PathSetting.PreMaxIterator;
            //Debug.Log("slot"+useSolt.Length);
            while (useSolt.Length>0 && maxIterator>0)
            {
                var soltGlobalIndex =  useSolt.Dequeue();
                completeSlot.Enqueue(soltGlobalIndex);
                ref var slot = ref spanSlot[soltGlobalIndex];
                //Debug.Log(slot.Index + "www" + soltGlobalIndex);

                //var requestState = handle.RequestStateLookup.AsRef(in slot.entity);
                //requestState.code = RequestStateCode.InPrograss;
                if (cancelQuery.Contains(slot.entity))
                {
                    definiteCancel.AddNoResize(slot.entity);
                    QuerySlot.ClearState(ref slot);
                    continue;
                }

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

    public struct RequestState : IComponentData
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
        public float stopDistance;
        public float3 exetern;
        public int areaMask;
        public int agentType;
        public float externLength;

        public readonly static NavAgentComponent Default = new NavAgentComponent() { angleSpeed = 360f, speed = 2f, stopDistance = 0.1f, agentType = 0, areaMask = -1, exetern = 1f };
        
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
        public static void Remove(NativeArray<Entity> entities, EntityManager entityManager)
        {
            entityManager.RemoveComponent<NavAgentComponent>(entities);
            entityManager.RemoveComponent<NavAgentTransform>(entities);
            entityManager.RemoveComponent<NavAgentPathSteer>(entities);
            entityManager.RemoveComponent<NavAgentLocation>(entities);
            entityManager.RemoveComponent<Request>(entities);
            entityManager.RemoveComponent<PathInfo>(entities);
            entityManager.RemoveComponent<WayPoint>(entities);
        }
    }
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
 
        public EntityTypeHandle EntityTypeHandle;
        public UnsafeParallelHashSet<Entity> CancelRequest;
        public UnsafeList<Entity> QueueRequestList;
        public UnsafeList<Entity> DefiniteCancel;

        public EntryQueryQueue.TypeHandle EntryQueryQueueTypeHandle;

        public QueryECSJob.TypeHandle QueryECSJobHandle;

        public GetQuerySlotOutputJob.TypeHandle GetQuerySlotOutputJobTypeHandle;

        private CheckAndEntryQuerySlotJob.TypeHandle CheckAndEntryQuerySlotJobHandle;
        private RemoveCancelRequestAndResetRequestState.TypeHandle RemoveCancelRequestAndResetRequestStateTypeHandle;
        public int allSlot;

        protected override void OnCreate()
        {
            endBuffer = World.GetExistingSystemManaged<EndSimulationEntityCommandBufferSystem>();
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

            QueueRequestList = new UnsafeList<Entity>(allSlot, Allocator.Persistent, NativeArrayOptions.ClearMemory);
            CancelRequest = new UnsafeParallelHashSet<Entity>(64, Allocator.Persistent);
            DefiniteCancel = new UnsafeList<Entity>(allSlot, Allocator.Persistent);

            RequestPathQuery_1 = GetEntityQuery(typeof(Request), typeof(RequestState)) ;

            EntryQueryQueueTypeHandle = new EntryQueryQueue.TypeHandle(ref CheckedStateRef);
            QueryECSJobHandle.Init(ref CheckedStateRef);
            GetQuerySlotOutputJobTypeHandle = new GetQuerySlotOutputJob.TypeHandle(ref CheckedStateRef);
            CheckAndEntryQuerySlotJobHandle = new CheckAndEntryQuerySlotJob.TypeHandle(ref CheckedStateRef);
            RemoveCancelRequestAndResetRequestStateTypeHandle = new RemoveCancelRequestAndResetRequestState.TypeHandle(ref CheckedStateRef);
        }

        protected override void OnUpdate()
        {
            EntityTypeHandle.Update(ref CheckedStateRef);
            EntryQueryQueueTypeHandle.Update(ref CheckedStateRef);
            QueryECSJobHandle.Update(ref CheckedStateRef);
            GetQuerySlotOutputJobTypeHandle.Update(ref CheckedStateRef);
            CheckAndEntryQuerySlotJobHandle.Update(ref CheckedStateRef);
            RemoveCancelRequestAndResetRequestStateTypeHandle.Update(ref CheckedStateRef);
            var maxEntryEntity = RequestPathQuery_1.CalculateEntityCountWithoutFiltering();
            if (QueueRequestList.Capacity - QueueRequestList.Length < maxEntryEntity)
            {
                QueueRequestList.SetCapacity(QueueRequestList.Capacity + maxEntryEntity);
            }
            
            Dependency = new EntryQueryQueue()
            {
                EntityHandle = EntityTypeHandle,
                QueueListWriter = QueueRequestList.AsParallelWriter(),
                CancelQuest = CancelRequest.AsParallelWriter(),
                typeHandle= EntryQueryQueueTypeHandle
            }.ScheduleParallel(RequestPathQuery_1, this.Dependency);

            Dependency = new CheckAndEntryQuerySlotJob()
            {
                typeHandle = CheckAndEntryQuerySlotJobHandle,
                allSlot = allSlot,
                process = PathSetting.Process,
                QueryListPtr = QueueRequestList.AsIntPtr(),
                querySlots = QuerySlotArray.GetBufferUnSafeIntPtr(),
                QueryECSArray = QueryECSArray.GetBufferUnSafeIntPtr(),
            }.Schedule(Dependency);

            Dependency = new QueryECSJob()
            {
                handle = QueryECSJobHandle,
                QueryECSPtr = QueryECSArray.GetBufferUnSafeIntPtr(),
                Process = PathSetting.Process,
                cancelQuery = CancelRequest.AsIntPtr(),
                DefiniteCancel = DefiniteCancel.AsParallelWriter()
            }.Schedule(PathSetting.Process, 1, Dependency);

            Dependency = new GetQuerySlotOutputJob()
            {
                AllSlot = allSlot,
                process = PathSetting.Process,
                QueryECS = QueryECSArray.GetBufferUnSafeIntPtr(),
                typeHandle = GetQuerySlotOutputJobTypeHandle,
                QuerySlotsArray = QuerySlotArray.GetBufferUnSafeIntPtr(),
                QuerySlotsOutputArray = QuerySlotOutputArray.GetBufferUnSafeIntPtr()
            }.Schedule(PathSetting.Process, 1, Dependency);

            Dependency = new RemoveCancelRequestAndResetRequestState() 
            { 
                CancelRequest = CancelRequest, 
                DefiniteCancelPtr = DefiniteCancel.AsIntPtr(), 
                typeHandle = RemoveCancelRequestAndResetRequestStateTypeHandle 
            }.Schedule(Dependency);

            endBuffer.AddJobHandleForProducer(Dependency);
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

            QueueRequestList.Dispose();
            CancelRequest.Dispose();
            DefiniteCancel.Dispose();
            base.OnDestroy();
        }

        [BurstCompile]
        public partial struct EntryQueryQueue : IJobChunk
        {
            public TypeHandle typeHandle;
            public UnsafeList<Entity>.ParallelWriter QueueListWriter;
            public UnsafeParallelHashSet<Entity>.ParallelWriter CancelQuest;
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
                    else
                    {
                        if (spanRequestState[index].code != RequestStateCode.Idle)
                        {
                            CancelQuest.Add(readOnlyEntity[index]);
                            spanRequestState[index].QuerySlotIndex = PathSetting.InvaildQuerySlot;
                        }
                        //else if (spanRequestState[index].code == RequestStateCode.InPrograss)
                        //{
                        //    QuerySolt[spanRequestState[index].QuerySlotIndex]
                        //}
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
        public partial struct CheckAndEntryQuerySlotJob :IJob
        {
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryListPtr;

            public int allSlot;
            public int process;

            [NativeDisableUnsafePtrRestriction()]
            public IntPtr querySlots;
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECSArray;

            public TypeHandle typeHandle;
            public void Execute()
            {

                ref var QueryList = ref QueryListPtr.As<UnsafeList<Entity>>();
                var allEntity = QueryList.Length;
                if (allEntity <= 0)
                {
                    return;
                }

                var spanQuerySlot = querySlots.AsSpan<QuerySlot>(allSlot);

                var que = QueryECSArray.AsSpan<QueryECS>(process);

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
                //Debug.Log("min"+min);
                var ecsIndex = 0;
              
                for (int i = 0; i < min; i++)
                {
                    //Debug.Log("x"+que[ecsIndex].freeSlot.Length);
                    //Debug.Log("y" + ecsIndex);
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
                    spanQuerySlot[soltIndex].entity = QueryList[i];
                    ref var rs = ref typeHandle.RequestStateLookup.AsRef(in spanQuerySlot[soltIndex].entity);
                    rs.QuerySlotIndex = soltIndex;
                    rs.code = RequestStateCode.InPrograss;
                }
                QueryList.RemoveRange(0, min);
            }

            public struct TypeHandle
            {
                [NativeDisableParallelForRestriction]
                [NativeDisableContainerSafetyRestriction]
                public ComponentLookup<RequestState> RequestStateLookup;
                public TypeHandle(ref SystemState systemState)
                {
                    RequestStateLookup = systemState.GetComponentLookup<RequestState>();
                }
                public void Update(ref SystemState systemState)
                {
                    RequestStateLookup.Update(ref  systemState);
                }
            }
        }
        [BurstCompile]
        public struct QueryECSJob : IJobParallelFor
        {
            [NativeDisableUnsafePtrRestriction()]
            public IntPtr QueryECSPtr;
            [ReadOnly]
            [NativeDisableUnsafePtrRestriction]
            public IntPtr cancelQuery;
            [WriteOnly]
            [NativeDisableUnsafePtrRestriction]
            [NativeDisableContainerSafetyRestriction]
            public UnsafeList<Entity>.ParallelWriter DefiniteCancel;
            public TypeHandle handle;
            [ReadOnly]
            public int Process;


            public void Execute(int index)
            {
                QueryECSPtr.AsSpan<QueryECS>(Process)[index].Update(
                    ref handle ,
                    cancelQuery,
                    in DefiniteCancel
                    );
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

                            var PathInfo = typeHandle.PathInfoLookup.GetRefRW(item.entity, false);
                            PathInfo.ValueRW = item.PathInfo;
                            //ecb.SetComponent<PathInfo>(item.Entity, item.PathInfo);
                            //wayPointDynamic.AddRange(spanSlotOutput[i].wayPoints.AsArray());
                            //Debug.Log(spanSlotOutput[i].wayPoints.Length + "query lengt1");
                            var waypointSpan = spanSlotOutput[i].wayPoints.AsReadOnlySpan();
                            var NavAgentPathSteerRW = typeHandle.NavAgentPathSteerLookup.GetRefRW(item.entity, false);
                            NavAgentPathSteer.Start(ref NavAgentPathSteerRW.ValueRW);
                            //Debug.Log(waypointSpan.Length + "query lengt");
                            wayPointDynamic.AddRange(waypointSpan);
                            //wayPointDynamic.AddRange(spanSlotOutput[i].wayPoints.AsArray())
                            QuerySlot.ClearState(ref item);
                            break;
                        case PathQueryStatus.Failure:
                            var wayPointDynamicFaile = typeHandle.WayPointLookup[item.entity];
                            wayPointDynamicFaile.Clear();
                            var PathInfoFail = typeHandle.PathInfoLookup.GetRefRW(item.entity, false);
                            PathInfoFail.ValueRW = item.PathInfo;
                            QuerySlot.ClearState(ref item);
                            break;
                        default:
                            break;
                    }

                }
            }
        }

        [BurstCompile]
        public struct RemoveCancelRequestAndResetRequestState : IJob
        {
            [NativeDisableUnsafePtrRestriction]
            public IntPtr DefiniteCancelPtr;
            public UnsafeParallelHashSet<Entity> CancelRequest;
            public TypeHandle typeHandle;
            public void Execute()
            {
                ref var DefiniteCancel =ref DefiniteCancelPtr.As<UnsafeList<Entity>>();
                for (int i = 0; i < DefiniteCancel.Length; i++)
                {
                    CancelRequest.Remove(DefiniteCancel[i]);
                    var state = typeHandle.RequestStateLookup.AsRef(in DefiniteCancel.ElementAt(i));
                    state.code = RequestStateCode.Idle;
                    state.QuerySlotIndex = PathSetting.InvaildQuerySlot;
                }
                DefiniteCancel.Clear();
            }

            public struct TypeHandle
            {
                public ComponentLookup<RequestState> RequestStateLookup;
                public TypeHandle(ref SystemState systemState)
                {
                    RequestStateLookup =  systemState.GetComponentLookup<RequestState>(false);
                }


                public void Update(ref SystemState systemState)
                {
                    RequestStateLookup.Update(ref systemState);
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
                    var deltaLength = (navAgentCom.speed * TimeDeltaTime);
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
                    navAgentTransform.rotation= mathEX.RotateTowards(in navAgentTransform.rotation, in targetRotation, navAgentCom.angleSpeed * TimeDeltaTime);
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