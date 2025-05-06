//using System;
//using System.Collections.Generic;
//using System.Linq;
//using System.Numerics;
//using System.Runtime.InteropServices;
//using System.Text;
//using System.Threading.Tasks;
//using VBeamRT.Raytracing.CPU.Common;

//namespace VBeamRT.Raytracing.CPU.Whitted.BVH;

//struct Material
//{
//    public Vec3 Color;
//    public float Roughness;

//    public Material(Vec3 color, float roughness)
//    {
//        Color = color;
//        Roughness = roughness;
//    }
//}

//abstract class ObjectPool
//{
//    protected ObjectPool(Array items)
//    {
//        Items = items;
//    }

//    public int Count { get; protected set; }
//    public int Capacity => Items.Length;

//    protected Array Items;
//}

//sealed class ObjectPool<T> : ObjectPool
//{
//    private T[] _typedItems => (T[])Items;
//    public ReadOnlySpan<T> TypedItems => ((T[])Items).AsSpan(0, Count);
//    public ObjectPool(int InitialCapacity = 16) : base(new T[InitialCapacity])
//    {

//    }

//    public int Add(T item)
//    {
//        var index = Count++;
//        if (Capacity <= Count)
//        {
//            Resize();
//        }
//        var items = _typedItems;
//        items[index] = item;
//        return index;
//    }

//    void Resize()
//    {
//        var items = _typedItems;
//        int newCapacity = (int)BitOperations.RoundUpToPowerOf2((uint)Capacity + 1);
//        Array.Resize(ref items, newCapacity);
//        Items = items;
//    }

//    public int Remove(int index)
//    {
//        var items = _typedItems;
//        int lastItem = --Count;
//        items[index] = items[lastItem];
//        return lastItem;
//    }
//}

//readonly struct ItemHandle
//{
//    public readonly int Id;
//    public readonly short Version;
//    public readonly short Type;

//    public ItemHandle(int id, short version, short type)
//    {
//        Id = id;
//        Version = version;
//        Type = type;
//    }
//}

//sealed class Scene
//{
//    public List<Material> Materials;
//    public List<PointLight> PointLights;
//    private int[] _itemMap;
//    private short[] _itemVersions;
//    private int _itemCount;
//    private Dictionary<Type, short> _typeKeys;
//    private List<int[]> _inverseItemMap;
//    private List<ObjectPool> _objectPools;

//    public Scene()
//    {
//        Materials = new List<Material>();
//        PointLights = new List<PointLight>();
//        _itemMap = new int[4];
//        _inverseItemMap = new List<int[]>();
//        _itemCount = 0;
//        _itemVersions = new short[4];
//        _objectPools = new List<ObjectPool>();
//    }

//    public ItemHandle Add<T>(T item) where T : ISceneItem
//    {
//        var key = GetOrAddTypeKey<T>();
//        var pool = GetOrAddPool<T>(key);
//        var index = pool.Add(item);

//        var invMap = _inverseItemMap[key];
//        int itemIndex = _itemCount++;
//        if (_itemCount >= _itemMap.Length)
//        {
//            int newCapacity = (int)BitOperations.RoundUpToPowerOf2((uint)_itemMap.Length + 1);
//            Array.Resize(ref _itemMap, newCapacity);
//            Array.Resize(ref _itemVersions, newCapacity);
//        }
//        if (invMap.Length != pool.Capacity)
//        {
//            Array.Resize(ref invMap, pool.Capacity);
//            _inverseItemMap[key] = invMap;
//        }
//        _itemMap[itemIndex] = index;
//        invMap[index] = itemIndex;
//        short version = _itemVersions[itemIndex] = (short)((~_itemVersions[itemIndex]) + 1);
//        return new ItemHandle(itemIndex, version, GetOrAddTypeKey<T>());
//    }

//    public bool Remove<T>(ItemHandle item) where T : ISceneItem
//    {
//        int index = _itemMap[item.Id];

//        int version = _itemVersions[item.Id];
//        if (version != item.Version || version <= 0 || GetOrAddTypeKey<T>() != item.Type)
//        {
//            return false;
//        }
//        int changedIndex = GetOrAddPool<T>().Remove(index);
//        int changedItem = _inverseItemMap[item.Type][changedIndex];
//        _itemMap[changedItem] = changedIndex;

//        _itemVersions[item.Id] = (short)~version;
//        return true;
//    }

//    private ObjectPool<T> GetOrAddPool<T>() where T : ISceneItem
//    {
//        var key = GetOrAddTypeKey<T>();
//        var cachedPool = TryGetPool(key);
//        if (cachedPool != null)
//        {
//            return (ObjectPool<T>)cachedPool;
//        }

//        var pool = new ObjectPool<T>();
//        _objectPools.Add(pool);

//        return pool;
//    }

//    private ObjectPool<T> GetOrAddPool<T>(short key) where T : ISceneItem
//    {
//        var cachedPool = TryGetPool(key);
//        if (cachedPool != null)
//        {
//            return (ObjectPool<T>)cachedPool;
//        }

//        var pool = new ObjectPool<T>();
//        _inverseItemMap.Add(new int[pool.Capacity]);
//        _objectPools.Add(pool);
//        return pool;
//    }

//    private ObjectPool? TryGetPool(short key)
//    {
//        if ((int)key < _objectPools.Count)
//        {
//            return _objectPools[key];
//        }
//        return null;
//    }

//    private short GetOrAddTypeKey<T>() where T : ISceneItem
//    {
//        ref var value = ref CollectionsMarshal.GetValueRefOrAddDefault(_typeKeys, typeof(T), out var exists);
//        if (!exists)
//        {
//            value = (short)_typeKeys.Count;
//        }
//        return value;
//    }

//    public HitInfo IntersectNearest(Ray ray)
//    {
//        foreach (var pool in _objectPools)
//        {
//            var spherePool = (ObjectPool<Sphere>)pool;
//            var spheres = spherePool.TypedItems;
//            for (int i = 0; i < spheres.Length; i++)
//            {
//                var sphere = spheres[i];
//                var info = sphere.HitTest(ray);
//            }

//        }
//    }
//}
