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
    public float distanceWeight = 2.0f;
    public float overlapPenalty = 0.2f;
    public float stepPenalty = 0.001f;
    public float distanceTolerance = 0.05f; // 허용 오차 (m 단위)

    public bool isAtIdealDistance = false; // 컨트롤러에서 읽기용

    [Header("Colliders")]
    public Collider myCollider;
    public string wallTag = "Wall";
    public string furnitureTag = "Furniture";

    [Header("References")]
    public Collider nearestWall;   // Inspector에서 확인용
    public float nearestWallDis;   // Inspector에서 확인용

    [HideInInspector] public FurnitureEnvController controller;

    Collider[] wallColliders;
    Bounds areaBounds;
    SimpleMultiAgentGroup group;
    float lastDistanceError;

   


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

    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. 현재 위치 (정규화)
        Vector3 p = transform.position;
        Vector2 span = new Vector2(areaBounds.size.x, areaBounds.size.z);
        Vector2 rel = new Vector2((p.x - areaBounds.min.x) / Mathf.Max(0.001f, span.x),
                                   (p.z - areaBounds.min.z) / Mathf.Max(0.001f, span.y));

        sensor.AddObservation(rel);

        // 2. 가장 가까운 벽과의 거리 및 방향
        GetNearestWallInfo(out float wallDist, out Vector3 wallDir);
        sensor.AddObservation(wallDist);           // 거리 오차
        sensor.AddObservation(new Vector2(wallDir.x, wallDir.z));       // 방향

        // 3. 가구 크기 (정규화)
        Vector3 size = myCollider.bounds.size;
        Vector3 normSize = new Vector3(
            size.x / areaBounds.size.x,
            size.y / areaBounds.size.y,
            size.z / areaBounds.size.z
        );
        sensor.AddObservation(normSize);

        // 4. 각자 이상 벽 거리
        sensor.AddObservation(targetWallDistance);

        // 5. 거리 오차 (자신의 이상 거리와 실제 거리 차이)
        float distanceError = wallDist - targetWallDistance;
        sensor.AddObservation(distanceError); // 음수면 너무 가까움, 양수면 너무 멂
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        int a = actions.DiscreteActions[0];
        Vector3 dir = Vector3.zero;
        if (a == 1) dir = transform.forward;
        if (a == 2) dir = -transform.forward;
        if (a == 3) dir = -transform.right;
        if (a == 4) dir = transform.right;
        if (a == 5) transform.Rotate(0f, 90f, 0f);
        if (a == 6) transform.Rotate(0f, -90f, 0f);
        TryMove(dir * moveSpeed * Time.fixedDeltaTime);

        // 벽과의 거리 계산
        GetNearestWallInfo(out float wallDist, out _);
        // 절댓값 오차
        float error = Mathf.Abs(wallDist - targetWallDistance);

        // 이상 거리 도달 여부 갱신
        isAtIdealDistance = error <= distanceTolerance;
        if(isAtIdealDistance)
            SetReward(1.0f); // 목표 도달시 큰 보상

        // 이전보다 오차가 줄어들면 양의 보상
        float delta = lastDistanceError - error;
        AddReward(delta * distanceWeight);

        AddReward(-stepPenalty);

        lastDistanceError = error;
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

        if (OverlapAt(p))
        {
            AddReward(-overlapPenalty);
            return;
        }
        // if (!OverlapAt(p)) 충돌 체크 후 이동
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
    void GetNearestWallInfo(out float distance, out Vector3 direction)
    {
        distance = float.MaxValue;
        direction = Vector3.zero;
        nearestWall = null;

        if (wallColliders == null || wallColliders.Length == 0)
            return;

        Vector3 c = myCollider.bounds.center;
        float best = float.MaxValue;
        Vector3 bestV = Vector3.zero;

        foreach (var w in wallColliders)
        {
            Vector3 wallPoint = w.ClosestPoint(c);
            Vector3 myPoint = myCollider.ClosestPoint(wallPoint);

            Vector3 v = new Vector3(wallPoint.x - myPoint.x, 0f, wallPoint.z - myPoint.z);
            float d = v.magnitude;

            if (d < best)
            {
                best = d;
                bestV = v;
                nearestWall = w; // Inspector에 표시
                nearestWallDis = best; // Inspector에 표시

            }
        }

        distance = best;
        direction = bestV.normalized;
    }
    // Debug용
    void OnDrawGizmosSelected()
    {
        if (nearestWall == null) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(myCollider.bounds.center, nearestWall.ClosestPoint(myCollider.bounds.center));
    }
}
