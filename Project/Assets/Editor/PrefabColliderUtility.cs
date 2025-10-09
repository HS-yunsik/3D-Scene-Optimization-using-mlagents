using UnityEngine;
using UnityEditor;

public class PrefabColliderUtility : EditorWindow
{
    [MenuItem("Tools/Prefabs/Auto Add Colliders (Selected Folder)")]
    static void AddCollidersToSelectedPrefabs()
    {
        string[] selectedFolders = Selection.assetGUIDs;
        if (selectedFolders.Length == 0)
        {
            Debug.LogError("⚠ Project 창에서 프리팹이 들어있는 폴더를 선택해주세요.");
            return;
        }

        int count = 0;
        foreach (string folderGuid in selectedFolders)
        {
            string folderPath = AssetDatabase.GUIDToAssetPath(folderGuid);

            // 해당 폴더에서 모든 프리팹 검색
            string[] prefabGuids = AssetDatabase.FindAssets("t:Prefab", new[] { folderPath });

            foreach (string guid in prefabGuids)
            {
                string path = AssetDatabase.GUIDToAssetPath(guid);
                GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(path);

                if (prefab == null) continue;

                // 프리팹을 임시 인스턴스로 열기
                GameObject instance = PrefabUtility.LoadPrefabContents(path);

                // BoxCollider 자동 적용
                ApplyAutoCollider(instance);

                // 프리팹 저장 후 닫기
                PrefabUtility.SaveAsPrefabAsset(instance, path);
                PrefabUtility.UnloadPrefabContents(instance);

                count++;
            }
        }

        Debug.Log($"✅ {count}개의 프리팹에 Collider 적용 완료");
    }

    static void ApplyAutoCollider(GameObject root)
    {
        BoxCollider boxCollider = root.GetComponent<BoxCollider>();
        if (boxCollider == null)
            boxCollider = root.AddComponent<BoxCollider>();

        Renderer[] renderers = root.GetComponentsInChildren<Renderer>();
        Collider[] colliders = root.GetComponentsInChildren<Collider>();

        if (renderers.Length == 0 && colliders.Length == 0)
            return;

        Bounds bounds = new Bounds(root.transform.position, Vector3.zero);

        if (renderers.Length > 0)
        {
            foreach (Renderer r in renderers)
            {
                bounds.Encapsulate(r.bounds);
            }
        }
        else if (colliders.Length > 0)
        {
            foreach (Collider c in colliders)
            {
                bounds.Encapsulate(c.bounds);
            }
        }

        Vector3 localCenter = root.transform.InverseTransformPoint(bounds.center);
        Vector3 localSize = bounds.size;

        boxCollider.center = localCenter;
        boxCollider.size = localSize;
    }
}
