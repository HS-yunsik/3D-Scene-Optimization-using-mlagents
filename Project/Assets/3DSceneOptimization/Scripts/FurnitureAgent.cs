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
    public float alignmentWeight = 0.5f; // 벽 정렬 보상 가중치
    public float overlapPenalty = 1f;
    public float stepPenalty = 0.001f;
    public float distanceTolerance = 0.2f; // 허용 오차 (m 단위)
    public float rotateTolerance = 0.9f; // 허용 오차 (내적값 -1~1)

    public bool isAtIdealDistance = false; // 컨트롤러에서 읽기용
    public bool isAtIdealRotate = false; // 컨트롤러에서 읽기용
    public bool isFrozen = false; // 목표 달성 후 움직이지 않도록
    public bool deadLock = false; // deadlock 상태
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
        GetNearestWallInfo(out float wallDist, out _);
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
        // 1. 현재 위치, 방향 (정규화)
        Vector3 p = transform.position;
        Vector2 span = new Vector2(areaBounds.size.x, areaBounds.size.z);
        Vector2 rel = new Vector2((p.x - areaBounds.min.x) / Mathf.Max(0.001f, span.x),
                                   (p.z - areaBounds.min.z) / Mathf.Max(0.001f, span.y));

        sensor.AddObservation(rel);
        sensor.AddObservation(new Vector2(transform.forward.x, transform.forward.z));

        Vector3 wallForward = nearestWall.transform.forward;
        sensor.AddObservation(new Vector2(wallForward.x, wallForward.z)); // XY 대신 XZ 평면

        // 3. 가구 크기 (정규화)
        Vector3 size = myCollider.bounds.size;
        Vector3 normSize = new Vector3(
            size.x / areaBounds.size.x,
            size.y / areaBounds.size.y,
            size.z / areaBounds.size.z
        );
        sensor.AddObservation(normSize);

        // 4. 각자 이상 벽 거리, 허용 오차
        sensor.AddObservation(targetWallDistance);
        sensor.AddObservation(distanceTolerance);
        sensor.AddObservation(rotateTolerance);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (isFrozen) return;

        int a = actions.DiscreteActions[0];
        Vector3 dir = Vector3.zero;
        if (a == 1) dir = transform.forward;
        if (a == 2) dir = -transform.forward;
        if (a == 3) dir = -transform.right;
        if (a == 4) dir = transform.right;
        if (a == 5) transform.rotation = Quaternion.Euler(0, 0, 0);      // +Z
        if (a == 6) transform.rotation = Quaternion.Euler(0, 180, 0);    // -Z
        if (a == 7) transform.rotation = Quaternion.Euler(0, 90, 0);     // +X
        if (a == 8) transform.rotation = Quaternion.Euler(0, -90, 0);    // -X

        if (dir != Vector3.zero)
            TryMove(dir * moveSpeed * Time.fixedDeltaTime);

        // 벽과의 거리 계산
        GetNearestWallInfo(out float wallDist, out _);

      

        // 벽 정렬 보상
        Vector3 wallForward = nearestWall.transform.forward;
        Vector3 agentForward = transform.forward;
        float alignment = Vector3.Dot(wallForward, agentForward);

        // 약한 스텝 감점
        AddReward(-stepPenalty);

        // 벽과 정렬  보상
        AddReward(alignment * alignmentWeight);

        // 거리 오차 계산
        float error = wallDist - targetWallDistance;
        AddReward((1f - Mathf.Clamp01(error))*distanceWeight);

        // 목표 조건 확인
        isAtIdealDistance = error <= distanceTolerance;
        isAtIdealRotate = alignment >= rotateTolerance;

        if (isAtIdealDistance && isAtIdealRotate && !isFrozen)
        {
            if (!OverlapAt(transform.position))
            {
                // 정상적으로 벽과 정렬, 겹치지 않음 → 성공
                SetReward(5.0f);
                isFrozen = true;
                controller.ReportSuccess(this);
            }
            else
            {
                // 겹쳐 있는 상태에서 멈추려 함 → 감점 및 Frozen 금지
                AddReward(-1.0f);
                isFrozen = false;
            }
        }
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

            // 2. 기본 탈출 방향 = 반대 방향
            Vector3 escapeDir = -delta.normalized;
            bool escaped = false;

            // 3. 작은 스텝으로 점진적으로 밀어내기 시도 (0.1m씩, 최대 10회)
            for (int i = 0; i < 10; i++)
            {
                Vector3 escapePos = transform.position + escapeDir * 0.1f * (i + 1);
                escapePos.x = Mathf.Clamp(escapePos.x, areaBounds.min.x + half.x, areaBounds.max.x - half.x);
                escapePos.z = Mathf.Clamp(escapePos.z, areaBounds.min.z + half.y, areaBounds.max.z - half.y);
                escapePos.y = areaBounds.center.y;

                if (!OverlapAt(escapePos))
                {
                    transform.position = escapePos;
                    AddReward(-0.1f);
                    escaped = true;
                    break;
                }
            }

            // 4. 여전히 못 빠져나왔다면 랜덤 회전 + 랜덤 위치 샘플
            if (!escaped)
            {
                AddReward(-1f); // 교착 페널티
                deadLock = true;

                // 랜덤 회전 (0,90,180,270 중 하나)
                float randomY = Random.Range(0, 4) * 90f;
                transform.rotation = Quaternion.Euler(0f, randomY, 0f);

                // 랜덤 이동 (범위 내)
                Vector3 randomPos = new Vector3(
                    Random.Range(areaBounds.min.x + half.x, areaBounds.max.x - half.x),
                    areaBounds.center.y,
                    Random.Range(areaBounds.min.z + half.y, areaBounds.max.z - half.y)
                );

                // 랜덤 위치가 비어있으면 그쪽으로 텔레포트
                if (!OverlapAt(randomPos))
                {
                    transform.position = randomPos;
                    AddReward(-0.5f);
                    deadLock = false;
                }
            }

            return;
        }
        // if (!OverlapAt(p)) 충돌 체크 후 이동
        transform.position = p;
    }

    public bool OverlapAt(Vector3 center)
    {
        var hits = Physics.OverlapBox(center, myCollider.bounds.extents, transform.rotation);
        foreach (var h in hits)
        {
            if (h == myCollider) continue;
            if (h.isTrigger) continue;
            if (h.CompareTag("ground")) continue;
            // if (h.CompareTag(wallTag)) return true;
            if (h.CompareTag(furnitureTag)) return true;
        }
        return false;
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

        if (wallColliders == null || wallColliders.Length == 0)
            return;

        Vector3 center = myCollider.bounds.center;
        float best = float.MaxValue;
        Vector3 bestV = Vector3.zero;
        Collider bestWall = null;

        foreach (var wall in wallColliders)
        {
            Vector3 wallPoint = wall.ClosestPoint(center);
            Vector3 dir = wallPoint - center;
            Vector2 dirVec2 = new Vector2(dir.x, dir.z);
            float dist = dirVec2.magnitude;

            if (dist < best)
            {
                best = dist;
                bestV = dir;
                bestWall = wall;
            }
        }
        // 가장 가까운 벽을 확정
        nearestWall = bestWall;
        nearestWallDis = best;

        // 방향 (XZ 평면 정규화)
        direction = new Vector3(bestV.x, 0, bestV.z).normalized;

        // Z방향 크기 보정 (콜라이더의 local extents 사용)
        float halfZ = myCollider.bounds.extents.z;
        distance = Mathf.Max(0f, best - halfZ);
    }
    // Debug용
    void OnDrawGizmosSelected()
    {
        if (nearestWall == null) return;
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(myCollider.bounds.center, nearestWall.ClosestPoint(myCollider.bounds.center));
    }
}
