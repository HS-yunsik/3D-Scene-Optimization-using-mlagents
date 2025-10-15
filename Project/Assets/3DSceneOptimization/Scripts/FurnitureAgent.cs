using System.Collections.Generic;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.Rendering;

public class FurnitureAgent : Agent
{
    [Header("Movement")]
    public float moveSpeed = 2f;
    [Header("Rewards")]
    public float targetWallDistance = 0.2f;
    public float distanceWeight = 2.0f;
    public float alignmentWeight = 0.5f; // 벽 정렬 보상 가중치
    public float overlapPenalty = 0.01f;
    public float stepPenalty = 0.001f;
    public float distanceTolerance = 0.2f; // 허용 오차 (m 단위)
    public float rotateTolerance = 0.9f; // 허용 오차 (내적값 -1~1)
    private float previousDistanceError; // 이전 스텝의 거리를 이용하여 올바른 방향 제시
    public float personalSpaceRadius = 2.0f;  // 이 반경 안에 들어오면 페널티 적용

    public bool isAtIdealDistance = false; // 컨트롤러에서 읽기용
    public bool isAtIdealRotate = false; // 컨트롤러에서 읽기용
    public bool isFrozen = false; // 목표 달성 후 움직이지 않도록
    public bool isoverlaped = false; // 겹침상태
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
    private Rigidbody rb; // Rigidbody를 저장할 변수 추가

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
        rb = GetComponent<Rigidbody>();
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
        isoverlaped = false;
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
        // 이동 액션 처리
        int moveAction = actions.DiscreteActions[0];
        Vector3 dir = Vector3.zero;
        if (moveAction == 1) dir = transform.forward;
        if (moveAction == 2) dir = -transform.forward;
        if (moveAction == 3) dir = -transform.right;
        if (moveAction == 4) dir = transform.right;

        if (dir != Vector3.zero)
            TryMove(dir * moveSpeed * Time.fixedDeltaTime);

        // 목표 도달 여부 판단을 위한 거리 계산 (보상 없이 계산만 수행)
        GetNearestWallInfo(out float finalWallDist, out _);
        float currentDistanceError = Mathf.Abs(finalWallDist - targetWallDistance);

        // 이전 스텝보다 목표 거리에 가까워졌다면 보상을, 멀어졌다면 페널티를 줍니다.
        float reward = (previousDistanceError - currentDistanceError) * distanceWeight;
        AddReward(reward);

        // 다음 스텝에서의 계산을 위해 현재 거리 오차를 저장합니다.
        previousDistanceError = currentDistanceError;


        isAtIdealDistance = currentDistanceError <= distanceTolerance;
        
        // 목표 거리에 도달하면 2단계(회전)으로 전환 (유지)
        if (isAtIdealDistance)
        {
            if(isoverlaped)
            {
                // 겹쳐있는 상태라면 페널티 주고 멈추지 않음
                AddReward(-0.5f);
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
        switch (rotateAction)
        {
            case 1: transform.rotation = Quaternion.Euler(0, 0, 0); break;   // +Z
            case 2: transform.rotation = Quaternion.Euler(0, 180, 0); break; // -Z
            case 3: transform.rotation = Quaternion.Euler(0, 90, 0); break;  // +X
            case 4: transform.rotation = Quaternion.Euler(0, -90, 0); break; // -X
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

    void TryMove(Vector3 delta)
    {
        Vector3 p = transform.position + delta;

        Vector2 half = HalfSizeXZ();
        p.x = Mathf.Clamp(p.x, areaBounds.min.x + half.x, areaBounds.max.x - half.x);
        p.z = Mathf.Clamp(p.z, areaBounds.min.z + half.y, areaBounds.max.z - half.y);
        p.y = areaBounds.center.y;

        rb.MovePosition(p); 
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

    private void OnTriggerStay(Collider other)
    {
        // 충돌한 대상이 가구나 벽이라면
        if (other.CompareTag(furnitureTag) || other.CompareTag(wallTag))
        {
            isoverlaped = true;
            AddReward(-overlapPenalty);
        }
    }
    private void OnTriggerExit(Collider other)
    {
        // 충돌한 대상이 가구나 벽이라면
        if (other.CompareTag(furnitureTag) || other.CompareTag(wallTag))
        {
            isoverlaped = false;
            AddReward(overlapPenalty * 5);
        }
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
