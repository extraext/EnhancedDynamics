using System;
using System.IO;
using UnityEngine;
using HarmonyLib;

namespace EnhancedDynamics
{
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class EnhancedDynamicsController : MonoBehaviour
    {
        public static EnhancedDynamicsController Instance { get; private set; }

        [Header("	trajectory & dynamic lag	")]
        public bool allowInAllModes = true;
        public float pitchInertiaDelay = 0.07f;
        public float rollInertiaDelay = 0.05f;
        public float yawInertiaDelay = 0.07f;
        public float progradeDriftWeight = 0.0f;

        [Header("		sway & translation		")]
        public float rollLateralSway = 0.091f;
        public float pitchVerticalSway = 0.091f;
        public float gLagSensitivity = 0.0850f;
        public float maxBackwardShift = 0.520f;
        public float maxForwardShift = 0.240f;

        [Header("	aerodynamic buffeting & shakes	")]
        public float buffetingStrength = 0.0013f;
        public float maxBuffetingOffset = 0.0195f;
        public float minQForBuffeting = 0.02f;
        public float gForceShakeStrength = 0.00104f;

        [Header("	 dynamic FOV & shocks    ")]
        public float maxFOVExtension = 0.0f;
        public float speedForMaxFOV = 200.0f;
        public float touchdownSensitivity = 0.0620f;
        public float throttleJoltSensitivity = 0.0520f;

        [Header("	keybinds   ")]
        public KeyCode emergencyResetKey = KeyCode.KeypadMultiply;
        public KeyCode toggleGuiKey = KeyCode.F8;
        public KeyCode toggleGuiKeyAlt = KeyCode.KeypadPlus;

        // internal state
        private static Harmony harmonyInstance = null;
        private const string LockID = "EnhancedDynamics_GUILock";
        private bool isGuiLocked = false;
        private FlightCamera.Modes lastFlightCamMode = FlightCamera.Modes.AUTO;

        private Vector3 lastAppliedOffset = Vector3.zero;
        private bool wasThirdPartyActive = false;

        private float currentPitchLag, pitchLagVel;
        private float currentRollLag, rollLagVel;
        private float currentYawLag, yawLagVel;
        private Vector3 localSwayOffset = Vector3.zero;
        private Vector3 swayVelocity = Vector3.zero;
        private Vector3 impulseVector = Vector3.zero;
        private Vector3 impulseVelocity = Vector3.zero;

        private Vector3 filteredAccel = Vector3.zero;
        private Vector3 filteredSrfVel = Vector3.zero;
        private Vector3 filteredAngVel = Vector3.zero;
        private Vector3 filteredCraftNose = Vector3.up;
        private float lastThrottle = 0.0f;
        private float smoothedDt = 0.02f;

        private Vector3 currentTerrainCorrection = Vector3.zero;
        private Vector3 terrainCorrectionVel = Vector3.zero;
		private static readonly RaycastHit[] terrainHits = new RaycastHit[16];

        private float shakeSeed = 0.0f;

        private float timeSinceReset = 999f;
        private const float ModeSettleDuration = 0.35f;

        // fov
        private float lastRecordedUserFOV = -1.0f;
        private float currentFOVOffset = 0.0f;

        // gui state
        private bool showGuiWindow = false;
        private int activeGuiTab = 0;
        private int lastGuiTab = 0;
        private Rect guiWindowRect = new Rect(60, 120, 360, 0);

        private void Awake()
        {
            Instance = this;
            LoadSettings();
            ModChecker.Init();

            if (harmonyInstance == null)
            {
                try
                {
                    harmonyInstance = new Harmony("com.enhanceddynamics.cameramod");
                    harmonyInstance.PatchAll();
                    Debug.Log("[EnhancedDynamics] Harmony patches initialized successfully.");
                }
                catch (Exception ex)
                {
                    Debug.LogError("[EnhancedDynamics] Harmony Patch Failed: " + ex.ToString());
                }
            }
        }

        private void Start()
        {
            ScreenMessages.PostScreenMessage("EnhancedDynamics Active | Press F8 to toggle GUI", 4.0f, ScreenMessageStyle.UPPER_CENTER);
        }

        private void OnEnable()
        {
            GameEvents.onVesselChange.Add(OnVesselChange);
            GameEvents.onCollision.Add(OnGroundCollision);
            GameEvents.onStageActivate.Add(OnStageActivated);
        }

        private void OnDisable()
        {
            GameEvents.onVesselChange.Remove(OnVesselChange);
            GameEvents.onCollision.Remove(OnGroundCollision);
            GameEvents.onStageActivate.Remove(OnStageActivated);
            UnlockInput();
            SaveSettings();
        }

        private void OnVesselChange(Vessel newVessel) => ResetCameraOffsets();
        private void OnStageActivated(int stage) => impulseVector += new Vector3(0f, 0f, 0.08f * throttleJoltSensitivity);

        private void OnGroundCollision(EventReport report)
        {
            Vessel active = FlightGlobals.ActiveVessel;
            if (active == null || report.origin == null || report.origin.vessel != active)
                return;

            Transform refTr = active.ReferenceTransform != null ? active.ReferenceTransform : active.transform;
            Vector3 rawVel = IsValidVector(active.srf_velocity) ? (Vector3)active.srf_velocity : Vector3.zero;

            Vector3 localImpactDir = refTr.InverseTransformDirection(-rawVel.normalized);
            float impactSpeed = Mathf.Max(Mathf.Abs((float)active.verticalSpeed), (float)active.srfSpeed * 0.6f);
            float shockMagnitude = Mathf.Clamp(impactSpeed * touchdownSensitivity * 1.8f, 0.030f, 0.32f);

            impulseVector += localImpactDir * shockMagnitude;
        }

        private float CalculateDynamicResponseDelay(float speedMS)
        {
            if (speedMS < 30.0f) return 1.00f;
            if (speedMS < 45.0f) return 0.90f;
            if (speedMS < 55.0f) return 0.80f;
            if (speedMS < 70.0f) return 0.65f;
            if (speedMS < 90.0f) return 0.45f;
            if (speedMS < 450.0f) return 0.25f;
            if (speedMS < 1000.0f) return 0.35f;
            if (speedMS < 1250.0f) return 0.45f;
            if (speedMS < 1450.0f) return 0.60f;
            return 1.00f;
        }

        private void Update()
        {
            if (Input.GetKeyDown(emergencyResetKey))
            {
                ResetCameraOffsets();
                ScreenMessages.PostScreenMessage("EnhancedDynamics: Offsets Reset", 1.5f, ScreenMessageStyle.LOWER_CENTER);
            }

            if (Input.GetKeyDown(toggleGuiKey) || Input.GetKeyDown(toggleGuiKeyAlt))
            {
                showGuiWindow = !showGuiWindow;
                if (!showGuiWindow) UnlockInput();
            }
        }

        public void RestoreUnmodifiedCamera(FlightCamera flightCam)
        {
            if (flightCam != null && lastAppliedOffset != Vector3.zero)
            {
                flightCam.transform.position -= lastAppliedOffset;
                lastAppliedOffset = Vector3.zero;
            }
        }

        public void ApplyCameraModifiers(FlightCamera flightCam)
        {
            // base safety checks
            if (flightCam == null || flightCam.mainCamera == null || MapView.MapIsEnabled || FlightDriver.Pause ||
                TimeWarp.CurrentRate > 4.0f || (TimeWarp.fetch != null && TimeWarp.fetch.Mode != TimeWarp.Modes.LOW && TimeWarp.CurrentRate > 1.0f) || 
                FlightGlobals.ActiveVessel == null || !FlightGlobals.ready || CameraManager.Instance == null)
            {
                return;
            }

            // 3rd party camera check (cameratools, hullcamvds, etc)
            if (ModChecker.IsThirdPartyCameraActive(flightCam))
            {
                if (!wasThirdPartyActive)
                {
                    RestoreUnmodifiedCamera(flightCam);
                    ResetCameraOffsets();
                    wasThirdPartyActive = true;
                }
                return;
            }

            if (wasThirdPartyActive)
            {
                wasThirdPartyActive = false;
                ResetCameraOffsets();
            }

            if (CameraManager.Instance.currentCameraMode != CameraManager.CameraMode.Flight)
            {
                return;
            }

            if (flightCam.mode != lastFlightCamMode)
            {
                lastFlightCamMode = flightCam.mode;
                ResetCameraOffsets();
                return;
            }

            if (!allowInAllModes && flightCam.mode != FlightCamera.Modes.CHASE)
            {
                return;
            }

            float rawDt = Mathf.Clamp(Time.deltaTime, 0.001f, 0.033f);
            smoothedDt = Mathf.Lerp(smoothedDt, rawDt, rawDt * 10.0f);
            float dt = smoothedDt;

            timeSinceReset += dt;
            float settleBlend = Mathf.Clamp01(timeSinceReset / ModeSettleDuration);

            Vessel vessel = FlightGlobals.ActiveVessel;
            bool isEVA = vessel.isEVA;
            Transform refTransform = vessel.ReferenceTransform != null ? vessel.ReferenceTransform : vessel.transform;

            Vector3 rawCraftNose = refTransform.up;
            if (filteredCraftNose == Vector3.zero) filteredCraftNose = rawCraftNose;
            filteredCraftNose = Vector3.Slerp(filteredCraftNose, rawCraftNose, dt * 10.0f);

            Vector3 rawAccel = IsValidVector(vessel.acceleration) ? (Vector3)vessel.acceleration : Vector3.zero;
            Vector3 rawSrfVel = IsValidVector(vessel.srf_velocity) ? (Vector3)vessel.srf_velocity : Vector3.zero;
            Vector3 rawAngVel = IsValidVector(vessel.angularVelocity) ? (Vector3)vessel.angularVelocity : Vector3.zero;

            if (vessel.Landed || vessel.Splashed)
            {
                Vector3 gravityVector = FlightGlobals.getGeeForceAtPosition(vessel.CoM);
                rawAccel -= gravityVector;
            }

            filteredSrfVel = Vector3.Lerp(filteredSrfVel, rawSrfVel, dt * 10.0f);
            filteredAccel = Vector3.Lerp(filteredAccel, rawAccel, dt * 8.0f);
            filteredAngVel = Vector3.Lerp(filteredAngVel, rawAngVel, dt * 12.0f);

            float speed = (float)vessel.srfSpeed;

            float currentResponseDelay = CalculateDynamicResponseDelay(speed);
            pitchInertiaDelay = currentResponseDelay;
            rollInertiaDelay = currentResponseDelay;
            yawInertiaDelay = currentResponseDelay;

            float pitchRate = filteredAngVel.x;
            float rollRate = filteredAngVel.y;
            float yawRate = -filteredAngVel.z;

            float targetPitchLag = 0.0f;
            float targetRollLag = 0.0f;
            float targetYawLag = 0.0f;

            if (!isEVA)
            {
                targetPitchLag = Mathf.Clamp(-pitchRate * 0.364f * Mathf.Rad2Deg, -5.85f, 5.85f);
                targetRollLag = Mathf.Clamp(rollRate * 0.520f * Mathf.Rad2Deg, -7.80f, 7.80f);
                targetYawLag = Mathf.Clamp(-yawRate * 0.234f * Mathf.Rad2Deg, -2.60f, 2.60f);

                if (vessel.Landed || vessel.Splashed)
                {
                    float pitchInput = -vessel.ctrlState.pitch;
                    targetPitchLag += pitchInput * 3.25f;
                }

                if (!vessel.Landed && speed > 10.0f && filteredSrfVel.sqrMagnitude > 1.0f)
                {
                    Vector3 localVel = refTransform.InverseTransformDirection(filteredSrfVel.normalized);
                    float aoaPitch = Mathf.Atan2(localVel.z, localVel.y) * Mathf.Rad2Deg;
                    float slipYaw = Mathf.Atan2(localVel.x, localVel.y) * Mathf.Rad2Deg;

                    targetPitchLag += Mathf.Clamp(aoaPitch * (progradeDriftWeight * 0.12f), -2.3f, 2.3f);
                    targetYawLag += Mathf.Clamp(slipYaw * (progradeDriftWeight * 0.12f), -1.5f, 1.5f);
                }
            }

            currentPitchLag = Mathf.SmoothDampAngle(currentPitchLag, targetPitchLag, ref pitchLagVel, pitchInertiaDelay, 120f, dt);
            currentRollLag = Mathf.SmoothDampAngle(currentRollLag, targetRollLag, ref rollLagVel, rollInertiaDelay, 120f, dt);
            currentYawLag = Mathf.SmoothDampAngle(currentYawLag, targetYawLag, ref yawLagVel, yawInertiaDelay, 120f, dt);

            Quaternion dynamicRotOffset = Quaternion.AngleAxis(currentPitchLag, Vector3.right) *
                                           Quaternion.AngleAxis(currentYawLag, Vector3.up) *
                                           Quaternion.AngleAxis(currentRollLag, Vector3.forward);

            if (!isEVA)
            {
                float currentThrottle = vessel.ctrlState.mainThrottle;
                float throttleDelta = (currentThrottle - lastThrottle) / dt;
                lastThrottle = currentThrottle;

                if (Mathf.Abs(throttleDelta) > 0.30f)
                {
                    float jolt = Mathf.Clamp(throttleDelta * 0.010f * throttleJoltSensitivity, -0.15f, 0.15f);
                    impulseVector += new Vector3(0, 0, jolt);
                }
            }

            impulseVector = Vector3.SmoothDamp(impulseVector, Vector3.zero, ref impulseVelocity, 0.08f, 10f, dt);

            Vector3 localAccel = refTransform.InverseTransformDirection(filteredAccel);

            float gLagZ = Mathf.Clamp((localAccel.y / 9.81f) * gLagSensitivity * 1.35f, -maxForwardShift, maxBackwardShift);
            float gLagX = Mathf.Clamp((-localAccel.x / 9.81f) * gLagSensitivity * 0.85f, -0.28f, 0.28f);
            float gLagY = Mathf.Clamp((-localAccel.z / 9.81f) * gLagSensitivity * 0.85f, -0.28f, 0.28f);

            float targetSwayX, targetSwayY, targetSwayZ;

            if (isEVA)
            {
                targetSwayX = Mathf.Clamp(gLagX + impulseVector.x, -0.32f, 0.32f);
                targetSwayY = Mathf.Clamp(gLagY + impulseVector.y, -0.32f, 0.32f);
                targetSwayZ = -(gLagZ + impulseVector.z);
            }
            else
            {
                float groundPitchBonus = (vessel.Landed && vessel.ctrlState.pitch < -0.1f) ? 0.065f : 0.0f;
                targetSwayX = Mathf.Clamp((-rollRate * rollLateralSway * 1.10f) + gLagX + impulseVector.x, -0.32f, 0.32f);
                targetSwayY = Mathf.Clamp((pitchRate * pitchVerticalSway * 0.85f) + groundPitchBonus + gLagY + impulseVector.y, -0.32f, 0.32f);
                targetSwayZ = -(gLagZ + impulseVector.z);
            }

            Vector3 targetSway = new Vector3(targetSwayX, targetSwayY, targetSwayZ);
            localSwayOffset = Vector3.SmoothDamp(localSwayOffset, targetSway, ref swayVelocity, 0.08f, 10f, dt);

            shakeSeed += dt * 10.0f;
            float qkPa = (float)vessel.dynamicPressurekPa;
            Vector3 localShake = Vector3.zero;

            if (!isEVA && (qkPa > minQForBuffeting || (!vessel.Landed && filteredAccel.magnitude > 12.0f)))
            {
                float qScale = Mathf.Clamp01(Mathf.Log10(qkPa + 1.0f) / 2.5f) * buffetingStrength;
                float gScale = vessel.Landed ? 0f : Mathf.Clamp01((filteredAccel.magnitude - 9.81f) / 25.0f) * gForceShakeStrength;
                float totalShake = qScale + gScale;

                float shakeX = (Mathf.PerlinNoise(shakeSeed, 0.0f) - 0.5f) * totalShake;
                float shakeY = (Mathf.PerlinNoise(0.0f, shakeSeed + 500f) - 0.5f) * totalShake;
                float shakeZ = (Mathf.PerlinNoise(shakeSeed + 250f, shakeSeed + 750f) - 0.5f) * totalShake;

                localShake = Vector3.ClampMagnitude(new Vector3(shakeX, shakeY, shakeZ), maxBuffetingOffset);
            }

            Vector3 worldSway = (flightCam.transform.right * localSwayOffset.x) + 
                                (flightCam.transform.up * localSwayOffset.y) + 
                                (filteredCraftNose * localSwayOffset.z);

            Vector3 worldShake = (flightCam.transform.right * localShake.x) + 
                                 (flightCam.transform.up * localShake.y) + 
                                 (filteredCraftNose * localShake.z);

            Vector3 stockCamPos = flightCam.transform.position;
            Quaternion stockCamRot = flightCam.transform.rotation;
            Vector3 visualOrigin = vessel.CoM;

            Vector3 targetCamPos = stockCamPos + (worldSway + worldShake) * settleBlend;

            Vector3 dirToCam = targetCamPos - visualOrigin;
            float distToCam = dirToCam.magnitude;
            float safeOffset = Mathf.Min(1.0f, distToCam * 0.35f);

            Vector3 targetCorrection = Vector3.zero;
            if (distToCam > safeOffset && distToCam > 0.1f)
            {
                Vector3 rayDir = dirToCam.normalized;
                Vector3 rayStart = visualOrigin + (rayDir * safeOffset);
                float rayDistance = distToCam - safeOffset;

                int terrainMask = 1 << 15;

                int hitCount = Physics.RaycastNonAlloc(rayStart, rayDir, terrainHits, rayDistance, terrainMask);
                float closestDist = float.MaxValue;
                bool foundValidTerrainHit = false;
                RaycastHit bestHit = default;

                for (int i = 0; i < hitCount; i++)
                {
                    RaycastHit h = terrainHits[i];
                    if (h.collider == null || h.collider.isTrigger) continue;

                    if (h.collider.GetComponentInParent<Part>() != null) continue;
                    if (h.collider.GetComponentInParent<Vessel>() != null) continue;
                    if (h.collider.transform.root == vessel.transform.root) continue;

                    if (h.distance < closestDist)
                    {
                        closestDist = h.distance;
                        bestHit = h;
                        foundValidTerrainHit = true;
                    }
                }

                if (foundValidTerrainHit)
                {
                    Vector3 desiredPos = bestHit.point + (bestHit.normal * 0.15f) - (rayDir * 0.15f);
                    targetCorrection = desiredPos - targetCamPos;
                }
            }

            currentTerrainCorrection = targetCorrection;
            terrainCorrectionVel = Vector3.zero;
            Vector3 finalCamPos = targetCamPos + currentTerrainCorrection;

            Quaternion blendedRotOffset = Quaternion.Slerp(Quaternion.identity, dynamicRotOffset, settleBlend);
            flightCam.transform.position = finalCamPos;
            flightCam.transform.rotation = stockCamRot * blendedRotOffset;

            lastAppliedOffset = finalCamPos - stockCamPos;

            Camera mainCam = flightCam.mainCamera;
            if (!isEVA && mainCam != null)
            {
                float scrollDelta = Input.GetAxis("Mouse ScrollWheel");
                if (lastRecordedUserFOV < 0f || Mathf.Abs(scrollDelta) > 0.001f)
                {
                    lastRecordedUserFOV = mainCam.fieldOfView - currentFOVOffset;
                }

                float speedRatio = Mathf.Clamp01(speed / speedForMaxFOV);
                float targetFOVOffset = speedRatio * maxFOVExtension;
                currentFOVOffset = Mathf.Lerp(currentFOVOffset, targetFOVOffset, dt * 2.5f);

                mainCam.fieldOfView = Mathf.Clamp(lastRecordedUserFOV + currentFOVOffset, 5.0f, 120.0f);
            }
        }

        private bool IsValidVector(Vector3 v)
        {
            return !float.IsNaN(v.x) && !float.IsNaN(v.y) && !float.IsNaN(v.z) &&
                   !float.IsInfinity(v.x) && !float.IsInfinity(v.y) && !float.IsInfinity(v.z);
        }

        private void ResetCameraOffsets()
        {
            if (lastRecordedUserFOV >= 0f)
            {
                Camera cam = FlightCamera.fetch != null ? FlightCamera.fetch.mainCamera : null;
                if (cam != null) cam.fieldOfView = lastRecordedUserFOV;
            }

            Vessel activeVessel = FlightGlobals.ActiveVessel;
            if (activeVessel != null)
            {
                lastThrottle = activeVessel.ctrlState.mainThrottle;
                Transform refTr = activeVessel.ReferenceTransform != null ? activeVessel.ReferenceTransform : activeVessel.transform;
                filteredCraftNose = refTr.up;
            }

            lastAppliedOffset = Vector3.zero;
            currentPitchLag = pitchLagVel = 0.0f;
            currentRollLag = rollLagVel = 0.0f;
            currentYawLag = yawLagVel = 0.0f;
            localSwayOffset = swayVelocity = Vector3.zero;
            impulseVector = impulseVelocity = Vector3.zero;
            filteredSrfVel = filteredAccel = filteredAngVel = Vector3.zero;
            currentTerrainCorrection = terrainCorrectionVel = Vector3.zero;
            currentFOVOffset = 0.0f;
            lastRecordedUserFOV = -1.0f;
            smoothedDt = 0.02f;
            timeSinceReset = 0.0f;
        }

        private string GetConfigFilePath()
        {
            string assemblyDir = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().Location);
            string directPath = Path.Combine(assemblyDir, "ed.cfg");
            if (File.Exists(directPath)) return directPath;

            return Path.Combine(KSPUtil.ApplicationRootPath, "GameData", "EnhancedDynamics", "PluginData", "ed.cfg");
        }

        private void LoadSettings()
        {
            try
            {
                string filePath = GetConfigFilePath();
                if (!File.Exists(filePath)) return;

                ConfigNode rootNode = ConfigNode.Load(filePath);
                if (rootNode == null) return;

                ConfigNode settingsNode = rootNode.GetNode("ENHANCED_DYNAMICS_SETTINGS") ?? rootNode;

                if (settingsNode.HasValue("allowInAllModes")) bool.TryParse(settingsNode.GetValue("allowInAllModes"), out allowInAllModes);
                if (settingsNode.HasValue("progradeDriftWeight")) float.TryParse(settingsNode.GetValue("progradeDriftWeight"), out progradeDriftWeight);
                if (settingsNode.HasValue("rollLateralSway")) float.TryParse(settingsNode.GetValue("rollLateralSway"), out rollLateralSway);
                if (settingsNode.HasValue("pitchVerticalSway")) float.TryParse(settingsNode.GetValue("pitchVerticalSway"), out pitchVerticalSway);
                if (settingsNode.HasValue("gLagSensitivity")) float.TryParse(settingsNode.GetValue("gLagSensitivity"), out gLagSensitivity);
                if (settingsNode.HasValue("maxBackwardShift")) float.TryParse(settingsNode.GetValue("maxBackwardShift"), out maxBackwardShift);
                if (settingsNode.HasValue("maxForwardShift")) float.TryParse(settingsNode.GetValue("maxForwardShift"), out maxForwardShift);
                if (settingsNode.HasValue("buffetingStrength")) float.TryParse(settingsNode.GetValue("buffetingStrength"), out buffetingStrength);
                if (settingsNode.HasValue("maxBuffetingOffset")) float.TryParse(settingsNode.GetValue("maxBuffetingOffset"), out maxBuffetingOffset);
                if (settingsNode.HasValue("minQForBuffeting")) float.TryParse(settingsNode.GetValue("minQForBuffeting"), out minQForBuffeting);
                if (settingsNode.HasValue("gForceShakeStrength")) float.TryParse(settingsNode.GetValue("gForceShakeStrength"), out gForceShakeStrength);
                if (settingsNode.HasValue("maxFOVExtension")) float.TryParse(settingsNode.GetValue("maxFOVExtension"), out maxFOVExtension);
                if (settingsNode.HasValue("speedForMaxFOV")) float.TryParse(settingsNode.GetValue("speedForMaxFOV"), out speedForMaxFOV);
                if (settingsNode.HasValue("touchdownSensitivity")) float.TryParse(settingsNode.GetValue("touchdownSensitivity"), out touchdownSensitivity);
                if (settingsNode.HasValue("throttleJoltSensitivity")) float.TryParse(settingsNode.GetValue("throttleJoltSensitivity"), out throttleJoltSensitivity);

                if (settingsNode.HasValue("emergencyResetKey")) emergencyResetKey = (KeyCode)Enum.Parse(typeof(KeyCode), settingsNode.GetValue("emergencyResetKey"));
                if (settingsNode.HasValue("toggleGuiKey")) toggleGuiKey = (KeyCode)Enum.Parse(typeof(KeyCode), settingsNode.GetValue("toggleGuiKey"));
                if (settingsNode.HasValue("toggleGuiKeyAlt")) toggleGuiKeyAlt = (KeyCode)Enum.Parse(typeof(KeyCode), settingsNode.GetValue("toggleGuiKeyAlt"));
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[EnhancedDynamics] Could not load settings.cfg: " + ex.Message);
            }
        }

        private void SaveSettings()
        {
            try
            {
                string filePath = GetConfigFilePath();
                string dir = Path.GetDirectoryName(filePath);
                if (!Directory.Exists(dir))
                {
                    Directory.CreateDirectory(dir);
                }

                ConfigNode rootNode = new ConfigNode();
                ConfigNode settingsNode = rootNode.AddNode("ENHANCED_DYNAMICS_SETTINGS");

                settingsNode.AddValue("allowInAllModes", allowInAllModes);
                settingsNode.AddValue("progradeDriftWeight", progradeDriftWeight);
                settingsNode.AddValue("rollLateralSway", rollLateralSway);
                settingsNode.AddValue("pitchVerticalSway", pitchVerticalSway);
                settingsNode.AddValue("gLagSensitivity", gLagSensitivity);
                settingsNode.AddValue("maxBackwardShift", maxBackwardShift);
                settingsNode.AddValue("maxForwardShift", maxForwardShift);
                settingsNode.AddValue("buffetingStrength", buffetingStrength);
                settingsNode.AddValue("maxBuffetingOffset", maxBuffetingOffset);
                settingsNode.AddValue("minQForBuffeting", minQForBuffeting);
                settingsNode.AddValue("gForceShakeStrength", gForceShakeStrength);
                settingsNode.AddValue("maxFOVExtension", maxFOVExtension);
                settingsNode.AddValue("speedForMaxFOV", speedForMaxFOV);
                settingsNode.AddValue("touchdownSensitivity", touchdownSensitivity);
                settingsNode.AddValue("throttleJoltSensitivity", throttleJoltSensitivity);
                settingsNode.AddValue("emergencyResetKey", emergencyResetKey.ToString());
                settingsNode.AddValue("toggleGuiKey", toggleGuiKey.ToString());
                settingsNode.AddValue("toggleGuiKeyAlt", toggleGuiKeyAlt.ToString());

                rootNode.Save(filePath);
                Debug.Log("[EnhancedDynamics] Saved settings to " + filePath);
            }
            catch (Exception ex)
            {
                Debug.LogWarning("[EnhancedDynamics] Could not save settings.cfg: " + ex.Message);
            }
        }

        private void OnGUI()
        {
            if (!showGuiWindow || MapView.MapIsEnabled || FlightGlobals.ActiveVessel == null)
            {
                UnlockInput();
                return;
            }

            Vector2 mousePos = new Vector2(Input.mousePosition.x, Screen.height - Input.mousePosition.y);
            if (guiWindowRect.Contains(mousePos))
            {
                if (!isGuiLocked)
                {
                    InputLockManager.SetControlLock(ControlTypes.ALL_SHIP_CONTROLS, LockID);
                    isGuiLocked = true;
                }
            }
            else
            {
                UnlockInput();
            }

            if (HighLogic.Skin != null) GUI.skin = HighLogic.Skin;

            if (activeGuiTab != lastGuiTab)
            {
                guiWindowRect.height = 0;
                lastGuiTab = activeGuiTab;
            }

            guiWindowRect = GUILayout.Window(992831, guiWindowRect, DrawTuningWindow, "<b>EnhancedDynamics Config</b>", GUILayout.Width(360));
        }

        private void UnlockInput()
        {
            if (isGuiLocked)
            {
                InputLockManager.RemoveControlLock(LockID);
                isGuiLocked = false;
            }
        }

        private void DrawTuningWindow(int windowID)
        {
            GUILayout.BeginVertical();

            allowInAllModes = GUILayout.Toggle(allowInAllModes, " Enable Effects in All Camera Modes");
            GUILayout.Space(8);

            activeGuiTab = GUILayout.Toolbar(activeGuiTab, new string[] { "Rotation", "Translation", "Shocks", "FOV" });
            GUILayout.Space(10);

            if (activeGuiTab == 0)
            {
                GUILayout.Label($"Prograde Drift Weight: <b>{progradeDriftWeight:F2}</b> <color=cyan>(Recommended: 0)</color>");
                progradeDriftWeight = GUILayout.HorizontalSlider(progradeDriftWeight, 0.0f, 0.35f);
            }
            else if (activeGuiTab == 1)
            {
                GUILayout.Label($"Roll Lateral Sway: <b>{rollLateralSway:F2} m</b>");
                rollLateralSway = GUILayout.HorizontalSlider(rollLateralSway, 0.0f, 0.35f);

                GUILayout.Label($"Pitch Vertical Sway (Up/Down): <b>{pitchVerticalSway:F2} m</b>");
                pitchVerticalSway = GUILayout.HorizontalSlider(pitchVerticalSway, 0.0f, 0.35f);

                GUILayout.Label($"G-Inertia / Braking Shift: <b>{gLagSensitivity:F3} m</b>");
                gLagSensitivity = GUILayout.HorizontalSlider(gLagSensitivity, 0.005f, 0.20f);
            }
            else if (activeGuiTab == 2)
            {
                GUILayout.Label($"Aerodynamic Buffeting Shake: <b>{buffetingStrength:F4}</b>");
                buffetingStrength = GUILayout.HorizontalSlider(buffetingStrength, 0.000f, 0.005f);

                GUILayout.Label($"High-G Turn Vibration: <b>{gForceShakeStrength:F4}</b>");
                gForceShakeStrength = GUILayout.HorizontalSlider(gForceShakeStrength, 0.000f, 0.005f);

                GUILayout.Label($"Landing Touchdown Shock: <b>{touchdownSensitivity:F3}</b>");
                touchdownSensitivity = GUILayout.HorizontalSlider(touchdownSensitivity, 0.005f, 0.70f);

                GUILayout.Label($"Throttle Response Jolt: <b>{throttleJoltSensitivity:F3}</b>");
                throttleJoltSensitivity = GUILayout.HorizontalSlider(throttleJoltSensitivity, 0.005f, 0.15f);
            }
            else if (activeGuiTab == 3)
            {
                GUIStyle warningStyle = new GUIStyle(GUI.skin.label) { richText = true, wordWrap = true };
                GUILayout.Label("<color=yellow>Warning: Not recommended, can cause visual bugs.</color>", warningStyle);
                GUILayout.Space(6);

                GUILayout.Label($"Speed FOV Expansion: <b>+{maxFOVExtension:F1}°</b>");
                maxFOVExtension = GUILayout.HorizontalSlider(maxFOVExtension, 0.0f, 8.0f);

                GUILayout.Label($"Speed For Max FOV: <b>{speedForMaxFOV:F0} m/s</b>");
                speedForMaxFOV = GUILayout.HorizontalSlider(speedForMaxFOV, 200.0f, 2000.0f);
            }

            GUILayout.Space(12);
            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Save Settings"))
            {
                SaveSettings();
                ScreenMessages.PostScreenMessage("EnhancedDynamics: Settings Saved", 1.5f, ScreenMessageStyle.LOWER_CENTER);
            }
            if (GUILayout.Button("Reset Offsets")) ResetCameraOffsets();
            GUILayout.EndHorizontal();

            GUILayout.EndVertical();
            GUI.DragWindow();
        }
    }

    [HarmonyPatch(typeof(FlightCamera), "LateUpdate")]
    public static class Patch_FlightCamera_LateUpdate
    {
        public static void Prefix(FlightCamera __instance)
        {
            if (EnhancedDynamicsController.Instance != null)
            {
                EnhancedDynamicsController.Instance.RestoreUnmodifiedCamera(__instance);
            }
        }

        public static void Postfix(FlightCamera __instance)
        {
            if (EnhancedDynamicsController.Instance != null)
            {
                EnhancedDynamicsController.Instance.ApplyCameraModifiers(__instance);
            }
        }
    }
}
