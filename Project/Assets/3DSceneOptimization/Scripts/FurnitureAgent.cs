using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class FurnitureAgent : Agent
{
    [Header("Movement")]
    public float moveSpeed = 2f;
    [Header("Rewards")]
    public float targetWallDistance = 0.2f;
    public float distanceWeight = 0.5f;
    public float overlapPenalty = 0.2f;
    public float stepPenalty = 0.001f;

    [Header("Colliders")]
    public Collider myCollider;
    public string wallTag = "Wall";
    public string furnitureTag = "Furniture";

    [HideInInspector] public FurnitureEnvController controller;

    Collider[] wallColliders;
    Bounds areaBounds;
    SimpleMultiAgentGroup group;
    float lastAbsError;

    public override void Initialize()
    {
        if (myCollider == null) myCollider = GetComponent<Collider>();

        if (controller == null)
            controller = GetComponentInParent<FurnitureEnvController>();
        if (controller == null)
        {
            Debug.LogWarning($"{name} has no controller assigned");
            return;
        }

        Transform parent = controller.transform;
        List<Collider> found = new List<Collider>();

        foreach (Transform child in parent.GetComponentsInChildren<Transform>())
        {
            if (child.CompareTag(wallTag))
            {
                Collider c = child.GetComponent<Collider>();
                if (c != null) found.Add(c);
            }
        }

        wallColliders = found.ToArray();

    }

    public void RegisterTo(SimpleMultiAgentGroup g)
    {
        group = g;
        g.RegisterAgent(this);
    }

    public void SetArea(Bounds b)
    {
        areaBounds = b;
    }

    public void Teleport(Vector3 p, Quaternion r)
    {
        transform.SetPositionAndRotation(p, r);
    }

    public void ClearStats()
    {
        lastAbsError = Mathf.Abs(CurrentWallError());
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 p = transform.position;
        Vector2 span = new Vector2(areaBounds.size.x, areaBounds.size.z);
        Vector2 rel = new Vector2((p.x - areaBounds.min.x) / Mathf.Max(0.001f, span.x),
                                   (p.z - areaBounds.min.z) / Mathf.Max(0.001f, span.y));
        sensor.AddObservation(rel);

        float err = CurrentWallError();
        sensor.AddObservation(err);

        Vector3 dirToWall = DirToNearestWallXZ();
        sensor.AddObservation(new Vector2(dirToWall.x, dirToWall.z));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int a = actions.DiscreteActions[0];
        Vector3 dir = Vector3.zero;
        if (a == 1) dir = transform.forward;
        if (a == 2) dir = -transform.forward;
        if (a == 3) dir = -transform.right;
        if (a == 4) dir = transform.right;

        TryMove(dir * moveSpeed * Time.fixedDeltaTime);

        float absErr = Mathf.Abs(CurrentWallError());
        float delta = lastAbsError - absErr;
        AddReward(distanceWeight * delta);
        lastAbsError = absErr;

        AddReward(-stepPenalty);

        if (OverlappingOthers())
            AddReward(-overlapPenalty);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var d = actionsOut.DiscreteActions;
        d[0] = 0;
        if (Input.GetKey(KeyCode.W)) d[0] = 1;
        if (Input.GetKey(KeyCode.S)) d[0] = 2;
        if (Input.GetKey(KeyCode.A)) d[0] = 3;
        if (Input.GetKey(KeyCode.D)) d[0] = 4;
    }

    void TryMove(Vector3 delta)
    {
        Vector3 p = transform.position + delta;

        Vector2 half = HalfSizeXZ();
        p.x = Mathf.Clamp(p.x, areaBounds.min.x + half.x, areaBounds.max.x - half.x);
        p.z = Mathf.Clamp(p.z, areaBounds.min.z + half.y, areaBounds.max.z - half.y);
        p.y = areaBounds.center.y;

        if (!OverlapAt(p))
            transform.position = p;
    }

    public bool OverlapAt(Vector3 center)
    {
        Vector3 he = myCollider.bounds.extents;
        Vector3 half = new Vector3(Mathf.Max(0.01f, he.x), Mathf.Max(0.01f, he.y), Mathf.Max(0.01f, he.z));

        var hits = Physics.OverlapBox(center, half, transform.rotation);
        foreach (var h in hits)
        {
            if (h == myCollider) continue;
            if (h.isTrigger) continue;
            if (h.CompareTag("ground")) continue;
            if (h.CompareTag(wallTag)) return true;
            if (h.CompareTag(furnitureTag)) return true;
        }
        return false;
    }

    public bool OverlappingOthers()
    {
        return OverlapAt(transform.position);
    }

    public Vector2 HalfSizeXZ()
    {
        var e = myCollider.bounds.extents;
        return new Vector2(e.x, e.z);
    }

    float CurrentWallError()
    {
        float d = NearestWallDistanceXZ();
        return d - targetWallDistance;
    }

    float NearestWallDistanceXZ()
    {
        if (wallColliders == null || wallColliders.Length == 0) return 1f;

        Vector3 c = myCollider.bounds.center;
        float best = float.MaxValue;

        foreach (var w in wallColliders)
        {
            Vector3 q = w.ClosestPoint(c);
            Vector3 v = new Vector3(q.x - c.x, 0f, q.z - c.z);
            float d = v.magnitude;
            if (d < best) best = d;
        }
        return best;
    }

    Vector3 DirToNearestWallXZ()
    {
        if (wallColliders == null || wallColliders.Length == 0) return Vector3.zero;

        Vector3 c = myCollider.bounds.center;
        float best = float.MaxValue;
        Vector3 bestV = Vector3.zero;

        foreach (var w in wallColliders)
        {
            // 내 콜라이더 표면에서 벽 표면까지 최단 거리 계산
            Vector3 wallPoint = w.ClosestPoint(myCollider.bounds.center);
            Vector3 myPoint = myCollider.ClosestPoint(wallPoint);

            Vector3 v = new Vector3(wallPoint.x - myPoint.x, 0f, wallPoint.z - myPoint.z);
            float d = v.magnitude;

            if (d < best)
            {
                best = d;
                bestV = v;
            }
        }
        return bestV.normalized;
    }
}
