using TMPro;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class WorldSpaceUIController : MonoBehaviour
{
    [SerializeField] private GameObject _iconPrefab;
    
    private Transform _mainCameraTransform;

    private void Start()
    {
        _mainCameraTransform = Camera.main.transform;
    }

    public GameObject DisplayDebugIcon(float number, float3 startPosition, float destroyTime = math.INFINITY)
    {
        var directionToCamera = (Vector3)startPosition - _mainCameraTransform.position;
        var rotationToCamera = Quaternion.LookRotation(directionToCamera, Vector3.up);
        var newIcon = Instantiate(_iconPrefab, startPosition, rotationToCamera, transform);
        var newIconText = newIcon.GetComponent<TextMeshProUGUI>();
        newIconText.text = $"<color=red>{number:#.00}</color>";
        if (destroyTime != math.INFINITY) {
            Destroy(newIcon, destroyTime);
        }
        return newIcon;
    }
}