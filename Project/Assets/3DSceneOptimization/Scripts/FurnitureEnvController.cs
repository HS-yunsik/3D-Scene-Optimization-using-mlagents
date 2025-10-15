using System.Collections.Generic;
using Unity.MLAgents;
using UnityEditor.PackageManager;
using UnityEngine;
public class FurnitureEnvController : MonoBehaviour
{
    [Header("Environment")]
    public GameObject ground;
    [Header("Agents")]
    public List<FurnitureAgent> agents = new List<FurnitureAgent>();

    [Header("Episode")]
    public int maxEnvironmentSteps = 5000;
    int successCount = 0;
    Collider groundCol;
    Bounds areaBounds;
    SimpleMultiAgentGroup group;
    int stepCount;


    // 에이전트 들을 담아놓는 빈 부모객체
    [SerializeField]
    Transform furnitureParent;
    void Start()
    {
        // 자동으로 Ground 객체 연결
        if (ground == null)
        {
            Transform child = transform.Find("Ground");
            if (child != null)
            {
                ground = child.gameObject;
                Debug.Log("Ground 객체 자동 연결 완료");
            }
            else
            {
                Debug.LogWarning("Ground라는 이름의 자식 객체를 찾을 수 없습니다");
            }
        }

        if (ground != null)
            areaBounds = ground.GetComponent<Collider>().bounds;

        group = new SimpleMultiAgentGroup();


        // agents 자동 등록
        agents.Clear();
        if (furnitureParent != null)
        {
            FurnitureAgent[] found = furnitureParent.GetComponentsInChildren<FurnitureAgent>();
            foreach (var a in found)
            {
                a.controller = this;
                agents.Add(a);
            }
        }

        foreach (var a in agents)
        {
            a.controller = this;
            a.RegisterTo(group);
            a.SetArea(areaBounds);
        }

        ResetScene();
    }

    void FixedUpdate()
    {
        stepCount += 1;
        if (stepCount >= maxEnvironmentSteps && maxEnvironmentSteps > 0)
        {
            group.GroupEpisodeInterrupted();
            ResetScene();
        }

        group.AddGroupReward(-0.5f / Mathf.Max(1, maxEnvironmentSteps));

    }

    public void ReportSuccess(FurnitureAgent a)
    {
        successCount++;
        if (successCount >= agents.Count)
        {
            Debug.Log($"모든 에이전트가 목표에 도달했습니다! stepCount={stepCount}");
            group.AddGroupReward(10.0f);
            group.EndGroupEpisode();
            ResetScene();
        }
    }

    public void ResetScene()
    {
        stepCount = 0;
        successCount = 0;
        foreach (var a in agents)
        {
            Vector3 p = SampleSpawn(a);

            // 0, 90, 180, 270 중 하나 선택
            float yRot = 90f * Random.Range(0, 4);
            Quaternion r = Quaternion.Euler(0f, yRot, 0f);
            a.isFrozen = false;
            a.isAtIdealDistance = false;
            a.isAtIdealRotate = false;
            a.Teleport(p, r);
        }
    }

    Vector3 SampleSpawn(FurnitureAgent a)
    {
        var b = areaBounds;
        var ext = a.HalfSizeXZ();

        float x = Random.Range(b.min.x + ext.x, b.max.x - ext.x);
        float z = Random.Range(b.min.z + ext.y, b.max.z - ext.y);
        return new Vector3(b.center.x, b.center.y, b.center.z);
    }

    public Bounds AreaBounds()
    {
        return areaBounds;
    }

}
