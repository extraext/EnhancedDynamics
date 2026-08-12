using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

namespace EnhancedDynamics
{
    public static class ModChecker
    {
        private static bool isInitialized = false;

        private static Type ctType = null;
        private static PropertyInfo ctFetchProp = null;
        private static FieldInfo ctFetchField = null;
        private static PropertyInfo ctInstanceProp = null;
        private static FieldInfo ctInstanceField = null;

        private static readonly List<PropertyInfo> ctBoolProps = new List<PropertyInfo>();
        private static readonly List<FieldInfo> ctBoolFields = new List<FieldInfo>();

        private static bool lastDetectionState = false;

        public static void Init()
        {
            if (isInitialized) return;
            isInitialized = true;

            BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.FlattenHierarchy;

            foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies())
            {
                Type[] types = null;
                try
                {
                    types = asm.GetTypes();
                }
                catch (ReflectionTypeLoadException ex)
                {
                    types = ex.Types;
                }
                catch { }

                if (types == null) continue;

                string asmName = asm.GetName().Name;

                // cameratools reflec setup
                if (asmName.Equals("CameraTools", StringComparison.OrdinalIgnoreCase))
                {
                    foreach (Type t in types)
                    {
                        if (t == null) continue;
                        if (t.Name.Equals("CamTools", StringComparison.OrdinalIgnoreCase) ||
                            t.Name.Equals("CameraTool", StringComparison.OrdinalIgnoreCase))
                        {
                            ctType = t;
                            break;
                        }
                    }

                    if (ctType != null)
                    {
                        ctFetchProp = ctType.GetProperty("fetch", flags) ?? ctType.GetProperty("Fetch", flags);
                        ctFetchField = ctType.GetField("fetch", flags) ?? ctType.GetField("Fetch", flags);
                        ctInstanceProp = ctType.GetProperty("Instance", flags) ?? ctType.GetProperty("instance", flags);
                        ctInstanceField = ctType.GetField("Instance", flags) ?? ctType.GetField("instance", flags);

                        HashSet<string> targetModeNames = new HashSet<string>(StringComparer.OrdinalIgnoreCase)
                        {
                            "cameraToolActive", "freeCamActive", "dogfightCamActive", 
                            "stationaryCamActive", "pathCamActive", "autoCamActive",
                            "isCameraToolActive", "isFreeCamActive", "isDogfightCamActive", 
                            "isStationaryCamActive", "isPathCamActive"
                        };

                        PropertyInfo[] props = ctType.GetProperties(flags);
                        foreach (var p in props)
                        {
                            if (p != null && p.DeclaringType == ctType && p.PropertyType == typeof(bool) && p.CanRead)
                            {
                                if (targetModeNames.Contains(p.Name)) ctBoolProps.Add(p);
                            }
                        }

                        FieldInfo[] fields = ctType.GetFields(flags);
                        foreach (var f in fields)
                        {
                            if (f != null && f.DeclaringType == ctType && f.FieldType == typeof(bool))
                            {
                                if (targetModeNames.Contains(f.Name)) ctBoolFields.Add(f);
                            }
                        }
                    }
                }
            }

            Debug.Log($"[EnhancedDynamics] ModChecker initialized. CameraTools mode props: {ctBoolProps.Count}, fields: {ctBoolFields.Count}.");
        }

        private static object GetCameraToolsInstance()
        {
            if (ctType == null) return null;

            try
            {
                if (ctFetchProp != null) { object val = ctFetchProp.GetValue(null, null); if (val != null) return val; }
                if (ctFetchField != null) { object val = ctFetchField.GetValue(null); if (val != null) return val; }
                if (ctInstanceProp != null) { object val = ctInstanceProp.GetValue(null, null); if (val != null) return val; }
                if (ctInstanceField != null) { object val = ctInstanceField.GetValue(null); if (val != null) return val; }

                return UnityEngine.Object.FindObjectOfType(ctType);
            }
            catch
            {
                return null;
            }
        }

        public static bool IsThirdPartyCameraActive(FlightCamera flightCam)
        {
            if (!isInitialized) Init();

            if (flightCam == null) return false;

            Camera mainCam = Camera.main;
            if (mainCam == null) return false;

            if (flightCam.mainCamera != null && mainCam != flightCam.mainCamera)
            {
                LogStateChange(true, $"Camera.main ('{mainCam.name}') differs from FlightCamera.mainCamera ('{flightCam.mainCamera.name}').");
                return true;
            }

            // cameratools activity checker
            if (ctType != null)
            {
                object ctInstance = GetCameraToolsInstance();
                if (ctInstance != null)
                {
                    for (int i = 0; i < ctBoolProps.Count; i++)
                    {
                        try
                        {
                            if ((bool)ctBoolProps[i].GetValue(ctInstance, null))
                            {
                                LogStateChange(true, $"CameraTools mode active: '{ctBoolProps[i].Name}'");
                                return true;
                            }
                        }
                        catch { }
                    }

                    for (int i = 0; i < ctBoolFields.Count; i++)
                    {
                        try
                        {
                            if ((bool)ctBoolFields[i].GetValue(ctInstance))
                            {
                                LogStateChange(true, $"CameraTools mode active: '{ctBoolFields[i].Name}'");
                                return true;
                            }
                        }
                        catch { }
                    }
                }
            }

            // hullcamvds state checker
            if (FlightGlobals.ActiveVessel != null && FlightGlobals.ActiveVessel.parts != null)
            {
                BindingFlags flags = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance | BindingFlags.Static | BindingFlags.FlattenHierarchy;
                var parts = FlightGlobals.ActiveVessel.parts;

                for (int i = 0; i < parts.Count; i++)
                {
                    var part = parts[i];
                    if (part == null || part.Modules == null) continue;

                    for (int m = 0; m < part.Modules.Count; m++)
                    {
                        var module = part.Modules[m];
                        if (module == null) continue;

                        Type modType = module.GetType();
                        string typeName = modType.Name.ToLowerInvariant();

                        if (typeName.Contains("hullcam") || typeName.Contains("dockingcam") || typeName.Contains("partcamera"))
                        {
                            FieldInfo[] fields = modType.GetFields(flags);
                            for (int f = 0; f < fields.Length; f++)
                            {
                                var fInfo = fields[f];
                                if (!fInfo.IsStatic && fInfo.FieldType == typeof(bool))
                                {
                                    string fName = fInfo.Name.ToLowerInvariant();
                                    if (fName == "isactivated" || fName == "cameraactive" || fName == "cameraenabled" || 
                                        fName == "iscameraactive" || fName == "iscamactive" || fName == "camactive")
                                    {
                                        try
                                        {
                                            if ((bool)fInfo.GetValue(module))
                                            {
                                                LogStateChange(true, $"Hullcam active flag '{fInfo.Name}'=true on module '{modType.Name}'");
                                                return true;
                                            }
                                        }
                                        catch { }
                                    }
                                }
                            }

                            PropertyInfo[] props = modType.GetProperties(flags);
                            for (int p = 0; p < props.Length; p++)
                            {
                                var pInfo = props[p];
                                if (pInfo.PropertyType == typeof(bool) && pInfo.CanRead)
                                {
                                    string pName = pInfo.Name.ToLowerInvariant();
                                    if (pName == "isactivated" || pName == "cameraactive" || pName == "cameraenabled" || 
                                        pName == "iscameraactive" || pName == "iscamactive" || pName == "camactive")
                                    {
                                        try
                                        {
                                            if ((bool)pInfo.GetValue(module, null))
                                            {
                                                LogStateChange(true, $"Hullcam active property '{pName}'=true on module '{modType.Name}'");
                                                return true;
                                            }
                                        }
                                        catch { }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            string mainCamName = mainCam.gameObject.name.ToLowerInvariant();
            if (mainCamName.Contains("hullcam") || 
                mainCamName.Contains("dockingcam") || 
                mainCamName.Contains("partcamera") || 
                mainCamName.Contains("rpm") || 
                mainCamName.Contains("rasterprop"))
            {
                LogStateChange(true, $"Active main camera name matched 3rd party mod: '{mainCam.gameObject.name}'");
                return true;
            }

            LogStateChange(false, "Stock FlightCamera active.");
            return false;
        }

        private static void LogStateChange(bool isThirdParty, string reason)
        {
            if (isThirdParty != lastDetectionState)
            {
                lastDetectionState = isThirdParty;
                if (isThirdParty)
                {
                    Debug.Log($"[EnhancedDynamics] Yielding camera control to 3rd party mod. Reason: {reason}");
                }
                else
                {
                    Debug.Log("[EnhancedDynamics] Regaining camera control (Stock Flight Camera active).");
                }
            }
        }
    }
}