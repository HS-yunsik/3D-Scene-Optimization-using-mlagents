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
    private float previousDistanceError; // 이전 스텝의 거리를 이용하여 올바른 방향 제시
    public float proximityPenaltyWeight = 0.5f; // 개인 공간 침범 페널티 가중치
    public float personalSpaceRadius = 2.0f;  // 이 반경 안에 들어오면 페널티 적용


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

    // Controller가 관리하는 다른 에이전트 목록
    private List<FurnitureAgent> otherAgents;

    public enum AgentPhase
    {
        Moving = 0,   // 1단계: 올바른 위치로 이동
        Rotating = 1, // 2단계: 올바른 방향으로 회전
        num = 2 // 페이즈 개수
    }
    [Header("Phase Control")]
    public AgentPhase currentPhase; // 현재 페이즈 (Inspector에서 확인용)
    // ==========================================================


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
        if (controller != null)
        {
            // 컨트롤러로부터 전체 에이전트 목록을 받아와서, 자기 자신은 제외
            otherAgents = controller.agents.Where(a => a != this).ToList();
        }
    }

    /// <summary>
    /// 에피소드가 시작될 때마다 호출됩니다. 상태를 초기화합니다.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        // ✨ 에피소드 시작 시 항상 1단계(이동)부터 시작하도록 설정
        currentPhase = AgentPhase.Moving;
        isFrozen = false;
        deadLock = false;
        isAtIdealDistance = false;
        isAtIdealRotate = false;

        GetNearestWallInfo(out float wallDist, out _);
        previousDistanceError = Mathf.Abs(wallDist - targetWallDistance);
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
        // ✨ 현재 페이즈 정보를 관측에 추가
        // 에이전트가 자신이 어떤 단계에 있는지 인지해야 학습이 가능합니다.
        sensor.AddOneHotObservation((int)currentPhase, (int)AgentPhase.num);

        // 1. 현재 위치, 방향 (정규화)
        Vector3 p = transform.position;
        Vector2 span = new Vector2(areaBounds.size.x, areaBounds.size.z);
        Vector2 rel = new Vector2((p.x - areaBounds.min.x) / Mathf.Max(0.001f, span.x),
                                      (p.z - areaBounds.min.z) / Mathf.Max(0.001f, span.y));

        sensor.AddObservation(rel);
        sensor.AddObservation(new Vector2(transform.forward.x, transform.forward.z));

        // 가장 가까운 벽 정보 업데이트 및 관측 추가
        GetNearestWallInfo(out _, out _);
        Vector3 wallForward = nearestWall != null ? nearestWall.transform.forward : Vector3.zero;
        sensor.AddObservation(new Vector2(wallForward.x, wallForward.z));

        // 가구 크기 (정규화)
        Vector3 size = myCollider.bounds.size;
        Vector3 normSize = new Vector3(
            size.x / areaBounds.size.x,
            size.y / areaBounds.size.y,
            size.z / areaBounds.size.z
        );
        sensor.AddObservation(normSize);

        // 거리 관련 값들
        // 방의 너비(x)를 기준으로 0~1 사이 값으로 정규화
        float maxDist = areaBounds.size.x;
        sensor.AddObservation(targetWallDistance / maxDist);
        sensor.AddObservation(distanceTolerance / maxDist);

        // 회전 허용 오차 (값이 -1~1 범위)
        sensor.AddObservation(rotateTolerance);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (isFrozen) return;

        // 매 스텝마다 약한 감점
        AddReward(-stepPenalty);

        // ✨ 현재 페이즈에 따라 행동과 보상 로직을 분리
        switch (currentPhase)
        {
            case AgentPhase.Moving:
                HandleMovingPhase(actions);
                break;
            case AgentPhase.Rotating:
                HandleRotatingPhase(actions);
                break;
        }
    }

    private void HandleMovingPhase(ActionBuffers actions)
    {
        // 이동 액션 처리 (기존과 동일)
        int moveAction = actions.DiscreteActions[0];
        Vector3 dir = Vector3.zero;
        if (moveAction == 1) dir = transform.forward;
        if (moveAction == 2) dir = -transform.forward;
        if (moveAction == 3) dir = -transform.right;
        if (moveAction == 4) dir = transform.right;

        if (dir != Vector3.zero)
            TryMove(dir * moveSpeed * Time.fixedDeltaTime);

        foreach (var other in otherAgents)
        {
            if (other.isFrozen || Vector3.Distance(transform.position, other.transform.position) > personalSpaceRadius)
            {
                continue;
            }
            float distance = Vector3.Distance(transform.position, other.transform.position);
            float penalty = -(1f / (distance * distance)) * proximityPenaltyWeight;
            AddReward(penalty);
        }

        // 목표 도달 여부 판단을 위한 거리 계산 (보상 없이 계산만 수행)
        GetNearestWallInfo(out float finalWallDist, out _);
        float finalCurrentError = Mathf.Abs(finalWallDist - targetWallDistance);
        isAtIdealDistance = finalCurrentError <= distanceTolerance;

        // 목표 거리에 도달하면 2단계(회전)으로 전환 (유지)
        if (isAtIdealDistance)
        {
            if (OverlapAt(transform.position))
            {
                AddReward(-overlapPenalty);
                return;
            }
            AddReward(1.0f); // 페이즈 전환 성공 보상
            currentPhase = AgentPhase.Rotating;
        }
    }


    /// <summary>
    /// 2단계: 회전 페이즈 처리
    /// </summary>
    private void HandleRotatingPhase(ActionBuffers actions)
    {
        // 이 페이즈에서는 더 이상 움직이지 않음

        // 회전 액션만 처리 (0:정지, 1:+Z, 2:-Z, 3:+X, 4:-X)
        int rotateAction = actions.DiscreteActions[1];
        bool hasRotated = true;
        switch (rotateAction)
        {
            case 1: transform.rotation = Quaternion.Euler(0, 0, 0); break;   // +Z
            case 2: transform.rotation = Quaternion.Euler(0, 180, 0); break; // -Z
            case 3: transform.rotation = Quaternion.Euler(0, 90, 0); break;  // +X
            case 4: transform.rotation = Quaternion.Euler(0, -90, 0); break; // -X
            default: hasRotated = false; break;
        }

        // 회전 후 겹치는지 확인
        if (hasRotated && OverlapAt(transform.position))
        {
            // 잘못된 회전으로 겹치게 되면 큰 페널티
            AddReward(-2.0f * overlapPenalty);
        }

        // --- 보상 로직 (오직 '정렬'에 대해서만) ---
        Vector3 wallForward = nearestWall.transform.forward;
        Vector3 agentForward = transform.forward;
        float alignment = Vector3.Dot(wallForward, agentForward);

        // 목표 정렬에 가까울수록 큰 보상
        AddReward(alignment * alignmentWeight);

        isAtIdealRotate = alignment >= rotateTolerance;

        // ✨ 최종 목표 달성 확인
        if (isAtIdealRotate)
        {
            // 성공! 큰 보상과 함께 에피소드 종료
            SetReward(5.0f);
            isFrozen = true;
            controller.ReportSuccess(this);

        }
    }


    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var d = actionsOut.DiscreteActions;
        // Branch 0: Movement
        d[0] = 0;
        if (Input.GetKey(KeyCode.W)) d[0] = 1;
        if (Input.GetKey(KeyCode.S)) d[0] = 2;
        if (Input.GetKey(KeyCode.A)) d[0] = 3;
        if (Input.GetKey(KeyCode.D)) d[0] = 4;

        // Branch 1: Rotation
        d[1] = 0;
        if (Input.GetKeyDown(KeyCode.UpArrow)) d[1] = 1;    // +Z
        if (Input.GetKeyDown(KeyCode.DownArrow)) d[1] = 2; // -Z
        if (Input.GetKeyDown(KeyCode.RightArrow)) d[1] = 3;  // +X
        if (Input.GetKeyDown(KeyCode.LeftArrow)) d[1] = 4;   // -X
    }

    // --- 아래 함수들은 기존 로직과 거의 동일 ---
    // TryMove, OverlapAt, HalfSizeXZ, GetNearestWallInfo, OnDrawGizmosSelected
    // (기존 코드와 동일하므로 생략)
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
        Collider bestWall = null;
        Vector3 bestWallPoint = Vector3.zero;
        float bestDistToCenter = float.MaxValue;

        // 1. 모든 벽을 순회하며 내 중심점과 가장 가까운 벽을 찾습니다.
        foreach (var wall in wallColliders)
        {
            Vector3 wallPoint = wall.ClosestPoint(center);
            float dist = Vector3.Distance(center, wallPoint);

            if (dist < bestDistToCenter)
            {
                bestDistToCenter = dist;
                bestWall = wall;
                bestWallPoint = wallPoint;
            }
        }

        if (bestWall == null) return;

        // ✨ --- 일반화된 거리 계산 (핵심 로직) --- ✨
        // 2. 위에서 찾은 벽 위의 지점(bestWallPoint)을 기준으로,
        //    이번엔 내 콜라이더에서 가장 가까운 지점(agentPoint)을 찾습니다.
        Vector3 agentPoint = myCollider.ClosestPoint(bestWallPoint);

        // 3. 두 표면 위의 지점 사이의 거리를 계산합니다. 이것이 최종 거리입니다.
        distance = Vector3.Distance(agentPoint, bestWallPoint);

        // --- Inspector 및 방향 벡터 업데이트 ---
        nearestWall = bestWall;
        nearestWallDis = distance; // 실제 표면 거리를 Inspector에 표시

        // 방향은 여전히 중심점 기준으로 계산하는 것이 안정적일 수 있습니다.
        direction = (bestWallPoint - center).normalized;
    }
    // Debug용
    void OnDrawGizmosSelected()
    {
        if (nearestWall == null || myCollider == null) return;

        Vector3 center = myCollider.bounds.center;
        Vector3 wallPoint = nearestWall.ClosestPoint(center);

        // 1. 가장 가까운 벽으로 향하는 노란색 선
        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(center, wallPoint);

        // 2. 계산된 목표 지점을 나타내는 녹색 구
        Vector3 dirToWall = (wallPoint - center).normalized;

        // 목표 지점 계산 (벽 표면으로부터의 거리)
        // 벽 표면과 가구 표면 사이의 거리가 targetWallDistance가 되어야 함
        float distFromCenterToWallCenter = Vector3.Distance(center, wallPoint);
        float furnitureExtent = myCollider.bounds.extents.z; // 가구의 절반 크기 (forward 방향 기준)

        // 실제 이동해야 하는 거리 = (현재 벽과의 거리) - (목표 거리) - (가구 크기 절반)
        // 이 계산은 가구의 forward 방향이 벽을 바라볼 때를 가정한 것
        // 좀 더 정확한 계산을 위해 벽과의 최단 거리(nearestWallDis)를 사용

        // 가장 가까운 벽 표면으로부터 targetWallDistance 만큼 떨어진 지점
        Vector3 targetPoint = wallPoint - (dirToWall * targetWallDistance);

        Gizmos.color = Color.green;
        Gizmos.DrawSphere(targetPoint, 0.1f);
    }
}
