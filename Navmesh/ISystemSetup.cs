using Unity.Collections;
using Unity.Entities;

public interface ISystemSetup
{
    ComponentType[] DepeondComponets { get; }
    ComponentType[] RequiredComponents { get; }

    void Remove(Entity entities);
    void Remove(NativeArray<Entity> entities);
    void Setup(Entity e);
    void Setup(NativeArray<Entity> e);
}