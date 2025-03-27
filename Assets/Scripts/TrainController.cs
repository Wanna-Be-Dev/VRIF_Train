//using NaughtyAttributes;
using System.Collections;
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Audio;
using TrainComponents;
using UnityEngine.Events;

namespace TrainComponents
{
    public class TrainController : MonoBehaviour
    {

        [Header("Track Parameters")]
        public int currentTrack = 0;
        public curve[] track;

        [Header("Movement Parameters")]
        public float startingPos = 0f;
        public bool isMoving = false;

        private Vector2 tLimit;

        [Header("Object Parameters")]
        public train headTrain;
        public train[] wagon;

        [Space(10)]
        [Header("Train Speed Parameters")]
        public float Maxspeed = 5f;

        private float direction = 0f;
        public float accelerationDuration = 3f;
        public float decelerationDuration = 3f;

        //[EnableIf("DebugMode")]
        public float speed = 0f;

        private float currentSpeed = 0f;
        private bool autoMovementisOn = false;

        [Space(10)]
        [Header("Train Control Parameters")]
        public float movementLever = 0f;
        public float gearBoxLever = 0f;

        [Space(10)]
        [Header("Train Audio")]
        [Range(0f, 1f)]
        public float volume = 1f;
        public AudioSource engineIdleAudio;
        public AudioSource engineGearAudio;
        private trainMovementState soundState = trainMovementState.Neutral;

        [Space(10)]
        [Header("Lookup Table Settings")]
        public int lookupTableResolution = 100; // Higher = smoother but heavier

        private float _t; // Current position along the curve (0-1)
        private LookupTable _lookupTable; // Precomputed curve data

        [Space(20)]
        [Header("Debug Mode")]
        public bool DebugMode = false;


        public event Action<TrainController> AutoMovementStopped;
        public event Action<TrainController, float> triggerReached;


        private void Start()
        {
            SetTrack(currentTrack);
            SetPosition(startingPos);
        }

        private void FixedUpdate()
        {
            if (track[currentTrack].controlPoints.Length < 2) return;

            if(!autoMovementisOn)
                ControlCheck();

            CheckForTriggers();
            //UpdateGages();

            if (isMoving)
            {
               
                currentSpeed = SmoothSpeed(ref currentSpeed, speed);

                //UpdateTrainSound();

                //movement logic
                UpdateTrainPosition(headTrain);

                foreach(var wagon in wagon)
                    UpdateWagonPosition(wagon);
            }
        }
        private float SmoothSpeed(ref float currentValue, float targetValue)
        {
            var duration =  targetValue != 0 ? accelerationDuration : decelerationDuration;
            if (Mathf.Abs(currentSpeed - speed) > 0.1f)
            {
                float t = Mathf.Clamp01(Time.fixedDeltaTime/ duration);
                return Mathf.Lerp(currentValue, targetValue, t);
            }
            else
                return targetValue;
        }

        // --- Train Logic ---
        private void UpdateTrainPosition(train main)
        {
            // Advance t based on arc-length distance
            float curveLength = _lookupTable.TotalLength;
            _t = Mathf.Clamp01(_t + (currentSpeed * Time.deltaTime) / curveLength);
            _t = Mathf.Clamp(_t, tLimit.x, tLimit.y);

            // Update Train position/rotation
            CurvePoint trainPoint = _lookupTable.GetPointAtDistance(_t * curveLength);
            main.body.position = Vector3.Lerp(main.body.position, trainPoint.Position, 0.1f);
            if (trainPoint.Tangent != Vector3.zero)
                main.body.rotation = Quaternion.Slerp(main.body.rotation, Quaternion.LookRotation(trainPoint.Tangent), 0.1f);

            // Rotate train wheels
            if (main.rotateWheels) RotateWheels(main.Wheels, trainPoint.Tangent);
        }

        // --- Wagon Logic ---
        private void UpdateWagonPosition(train wagon)
        {
            if (!wagon.body) return;

            // Get wagon's target distance behind train
            float targetDistance = Mathf.Max(0, (_t * _lookupTable.TotalLength) - wagon.wagonDistance);
            CurvePoint wagonPoint = _lookupTable.GetPointAtDistance(targetDistance);

            // Update Wagon position/rotation
            wagon.body.position = Vector3.Lerp(wagon.body.position, wagonPoint.Position, 0.1f);
            if (wagonPoint.Tangent != Vector3.zero)
                wagon.body.rotation = Quaternion.Slerp(wagon.body.rotation, Quaternion.LookRotation(wagonPoint.Tangent), 0.1f);

            // Rotate wagon wheels
            if (wagon.rotateWheels) RotateWheels(wagon.Wheels, wagonPoint.Tangent);
        }
        private void UpdateGages()
        {
            /*var speedPercentage = currentSpeed / Maxspeed;
            var rotAngle = speedometer.MaxValue * Mathf.Abs(speedPercentage);
            var targetRot = new Vector3(0, 0, rotAngle);
            speedometer.gameObject.transform.localRotation = Quaternion.Slerp(speedometer.gameObject.transform.localRotation, Quaternion.Euler(targetRot), Time.fixedDeltaTime);*/
        }

        private void RotateWheels(Transform[] wheels, Vector3 tangent)
        {
            foreach (Transform wheel in wheels)
            {
                wheel.rotation = Quaternion.LookRotation(tangent);
            }
        }

        private void CheckForTriggers()
        {
            for (int i = 0; i < track[currentTrack].triggers.Count; i++)
            {
                var trigger = track[currentTrack].triggers[i];

                if (trigger.isActive && Mathf.Abs(trigger.tvalue - _t) <= 0.001f)//Redundancy check
                {
                    triggerReached?.Invoke(this, trigger.tvalue);
                    trigger.isActive = false;
                    track[currentTrack].triggers[i] = trigger;
                }
            }
        }
        // --- Automation Movement ---
        private IEnumerator AutoMovement(float duration, float _speed)
        {
            autoMovementisOn = true;
            var movementTimer = duration;
            speed = _speed;
            while (movementTimer > 0)
            {
                movementTimer -= Time.deltaTime;
                if (movementTimer <= accelerationDuration) speed = 0;
                yield return null;
            }
            AutoMovementStopped?.Invoke(this);
            autoMovementisOn = false;

            if (DebugMode)
                Debug.Log("train stopped! time:" + duration +"speed:"+ speed);
        }
        //TODO: Make smoother Transition 

        private void UpdateTrainSound()
        {

            var speedPercentage = Mathf.Abs(currentSpeed) / Maxspeed;

            var idleVolume = volume - speedPercentage * volume;
            var gearVolume = speedPercentage * volume;

            var idlePitch = (0.14f * speedPercentage)+0.96f; // Change value for diffrent midle pitch 0.96f
            var gearPitch = (0.05f * speedPercentage)+0.95f; // Change value for diffrent midle pitch 0.95f

            engineIdleAudio.pitch = Mathf.Lerp(engineIdleAudio.pitch, idlePitch, Time.fixedDeltaTime);
            engineIdleAudio.volume = Mathf.Lerp(engineIdleAudio.volume, idleVolume, Time.fixedDeltaTime);

            engineGearAudio.pitch = Mathf.Lerp(engineGearAudio.pitch, gearPitch, Time.fixedDeltaTime);
            engineGearAudio.volume = Mathf.Lerp(engineGearAudio.volume, gearVolume, Time.fixedDeltaTime);
        }
        // --- Editor ---
        private void OnDrawGizmos()
        {
            //Drawlines
            for (int i = 0; i < track.Length; i++)
            {
                if (track[i].gizmosCurve)
                {
                    if (track[i].controlPoints != null && track[i].controlPoints.Length > 2)
                        DrawCurveGizmos(track[i].controlPoints, 100);
                }
            }
                   
            //Tackpoints
            if (track[currentTrack].gizmosPoints)
            {
                Gizmos.color = UnityEngine.Color.red;
                foreach (Transform point in track[currentTrack].controlPoints)
                {
                    if (point != null)
                        Gizmos.DrawSphere(point.position, 0.25f);
                }
            }

            //tLimit points
            if (track[currentTrack].gizmosLimits)
            {
                Gizmos.color = UnityEngine.Color.blue;
                DrawPointOnCurve(currentTrack, track[currentTrack].tLimit.x);
                DrawPointOnCurve(currentTrack, track[currentTrack].tLimit.y);
            }
            //Triggers
            if (track[currentTrack].gizmosTriggers)
            {
                Gizmos.color = UnityEngine.Color.yellow;
                foreach (var track in track[currentTrack].triggers)
                {
                    DrawPointOnCurve(currentTrack, track.tvalue);
                }
            }
            
        }
        public void DrawPointOnCurve(int trackIndex, float tarPosition)
        {
            var lookupTable = new LookupTable(track[trackIndex].controlPoints, lookupTableResolution);
            CurvePoint _target = lookupTable.GetPointAtDistance(tarPosition * lookupTable.TotalLength);
            Gizmos.DrawCube(_target.Position, new Vector3(0.1f, 0.5f, 0.5f));
        }

        private void DrawCurveGizmos(Transform[] points, int resolution)
        {
            if (points.Length < 2) return;

            Gizmos.color = UnityEngine.Color.green;
            Vector3 previousPoint = CalculateBezierPoint(0f, points);

            for (int i = 1; i <= resolution; i++)
            {
                float t = i / (float)resolution;
                Vector3 nextPoint = CalculateBezierPoint(t, points);
                Gizmos.DrawLine(previousPoint, nextPoint);
                previousPoint = nextPoint;
            }

        }
        private Vector3 CalculateBezierPoint(float t, Transform[] points)
        {
            Vector3[] temp = new Vector3[points.Length];
            for (int i = 0; i < points.Length; i++) temp[i] = points[i].position;

            for (int r = 1; r < points.Length; r++)
                for (int i = 0; i < points.Length - r; i++)
                    temp[i] = Vector3.Lerp(temp[i], temp[i + 1], t);

            return temp[0];
        }
        private bool ResetTrigger(int _trackIndex, float _triggerPosition, bool _activityState)
        {
            for (int i = 0; i < track[_trackIndex].triggers.Count; i++)
            {
                var trigger = track[_trackIndex].triggers[i];

                if (trigger.tvalue == _triggerPosition)
                {
                    trigger.isActive = _activityState;
                    track[currentTrack].triggers[i] = trigger;
                    return true;
                }
            }
            return false;
        }
        private void ControlCheck()
        {
            if (!DebugMode)
                speed = speed * direction;
        }
        // --- Set Methods ---
        public void SetIsMoving(bool state)
        {
            isMoving = state;
            direction = 0f;
            speed = 0f;
            currentSpeed = 0f;
        }
        public void SetTrackButton()
        {
            if (currentTrack == track.Length-1)
                currentTrack = 0;
            else
                currentTrack++;
            SetTrack(currentTrack);
        }
        public void SetSpeed(float angle)
        {
            speed = Mathf.Clamp(angle,0,Maxspeed);
        }
        public void SetDirection(float angle)
        {
            direction = Mathf.Clamp(angle, -1, 1);
        }
        public void SetTrack(int index)
        {
            currentTrack = index;
            _lookupTable = new LookupTable(track[index].controlPoints, lookupTableResolution);
            tLimit = track[index].tLimit;
        }
        public void DisableMovement(bool state)
        {
            isMoving = state;
        }
        public void SetPosition(float pos)
        {
            _t = Mathf.Clamp(pos,0, 1);
        }
        public void SetPositionPercentage(float pos)
        {
            pos = pos / 100;
            _t = Mathf.Clamp(pos, 0, 1);
        }
        public void StartAutomaticMovement(float _duration , float _speed)
        {
            StartCoroutine(AutoMovement(_duration, _speed));
        }
        public void AddTrackTrigger(int trackIndex, float triggerPosition, bool activityState)
        {
            if (!ResetTrigger(trackIndex, triggerPosition, activityState))
            { 
                triggerlimit newTrigger = new triggerlimit();
                newTrigger.isActive = activityState;
                newTrigger.tvalue = Mathf.Clamp(triggerPosition, 0, 1);
                track[trackIndex].triggers.Add(newTrigger);
            }
        }
        public enum trainMovementState
        {
            Default,
            Neutral,
            Gear
        }
    }
    // Bezier quadratic formula: C(t)=(1−t)^2P0+2t(1−t)P1+t^2P2
    public class LookupTable
    {
        public List<CurvePoint> Points { get; } = new List<CurvePoint>();
        public float TotalLength { get; private set; }

        public LookupTable(Transform[] controlPoints, int resolution)
        {
            PrecomputeCurve(controlPoints, resolution);
        }

        private void PrecomputeCurve(Transform[] controlPoints, int resolution)
        {
            Points.Clear();
            TotalLength = 0f;

            Vector3 prevPosition = CalculateBezierPoint(0f, controlPoints);
            Points.Add(new CurvePoint(prevPosition, Vector3.zero, 0f));

            for (int i = 1; i <= resolution; i++)
            {
                float t = i / (float)resolution;
                Vector3 position = CalculateBezierPoint(t, controlPoints);
                Vector3 tangent = CalculateTangent(t, controlPoints, 0.01f);

                float segmentLength = Vector3.Distance(prevPosition, position);
                TotalLength += segmentLength;

                Points.Add(new CurvePoint(position, tangent, TotalLength));
                prevPosition = position;
            }
        }

        public CurvePoint GetPointAtDistance(float targetDistance)
        {
            targetDistance = Mathf.Clamp(targetDistance, 0, TotalLength);

            // Binary search for efficiency
            int low = 0;
            int high = Points.Count - 1;

            while (low < high)
            {
                int mid = (low + high) / 2;
                if (Points[mid].Distance < targetDistance)
                    low = mid + 1;
                else
                    high = mid;
            }

            // Linear interpolate between points
            if (low > 0 && low < Points.Count)
            {
                CurvePoint a = Points[low - 1];
                CurvePoint b = Points[low];
                float lerp = (targetDistance - a.Distance) / (b.Distance - a.Distance);
                return CurvePoint.Lerp(a, b, lerp);
            }

            return Points[low];
        }

        private Vector3 CalculateBezierPoint(float t, Transform[] points)
        {
            Vector3[] temp = new Vector3[points.Length];
            for (int i = 0; i < points.Length; i++) temp[i] = points[i].position;

            for (int r = 1; r < points.Length; r++)
                for (int i = 0; i < points.Length - r; i++)
                    temp[i] = Vector3.Lerp(temp[i], temp[i + 1], t);

            return temp[0];
        }

        private Vector3 CalculateTangent(float t, Transform[] points, float delta)
        {
            Vector3 p1 = CalculateBezierPoint(t, points);
            Vector3 p2 = CalculateBezierPoint(t + delta, points);
            return (p2 - p1).normalized;
        }
    }
    //Curvature struct
    public struct CurvePoint
    {
        public Vector3 Position;
        public Vector3 Tangent;
        public float Distance;

        public CurvePoint(Vector3 pos, Vector3 tan, float dist)
        {
            Position = pos;
            Tangent = tan;
            Distance = dist;
        }

        public static CurvePoint Lerp(CurvePoint a, CurvePoint b, float t)
        {
            return new CurvePoint(
                Vector3.Lerp(a.Position, b.Position, t),
                Vector3.Lerp(a.Tangent, b.Tangent, t),
                Mathf.Lerp(a.Distance, b.Distance, t)
            );
        }
    }

}
[System.Serializable]
public class curve
{
    public Vector2 tLimit = new Vector2(0f, 1f);
    public Transform[] controlPoints;
    public List<triggerlimit> triggers = new List<triggerlimit>();
    //Disable gizmos
    [Header("Gizmos settings")]
    public bool gizmosPoints = true;
    public bool gizmosCurve = true;
    public bool gizmosLimits = true;
    public bool gizmosTriggers= true;
}
[System.Serializable]
public struct triggerlimit
{
    public bool isActive;
    [Range(0, 1)]
    public float tvalue;
    public UnityEvent TriggerAchieved;
}
[System.Serializable]
public class train
{
    public Transform body;
    public float wagonDistance = 2f;
    public bool rotateWheels = true;
    public Transform[] Wheels;
}