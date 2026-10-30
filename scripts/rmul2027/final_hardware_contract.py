"""Final 1029 sentry hardware inventory and fail-closed deployment preflight.

This tool writes interface templates only; it never launches ROS, drives hardware,
or changes the adopted C067 runtime.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import yaml

HERE=Path(__file__).resolve().parent
DEFAULT_PROFILE=HERE/"config/hardware/rmul2027_sentry_final_20261029.yaml"

def inventory_errors(profile):
    errors=[]
    def require(value,message):
        if not value:errors.append(message)
    sensors=profile.get("sensors",{})
    lidars=sensors.get("lidars",[])
    require(len(lidars)==2,"Final report requires exactly two MID-360 devices")
    require(all(x.get("model")=="MID-360" for x in lidars),"Both lidars must be MID-360; RPLidar is not the final second device")
    for key in ("id","frame","cloud_topic","imu_topic"):
        values=[x.get(key) for x in lidars]
        require(all(values) and len(set(values))==len(values),"Distinct per-device "+key+" required")
    require({x.get("id") for x in lidars}=={"primary","auxiliary"},"Primary and auxiliary lidar roles must be explicit")
    require(sensors.get("camera",{}).get("quantity")==1,"One industrial camera must remain in the inventory")
    require(profile.get("compute",{}).get("platform")=="Jetson_Orin","Jetson Orin must remain in the inventory")
    require(profile.get("power",{}).get("supercapacitor",{}).get("present") is True,"Supercapacitor must remain in the inventory")
    chassis=profile.get("chassis",{})
    require(chassis.get("drivetrain")=="omni_wheel" and chassis.get("suspension")=="independent","Sentry uses omni wheels and independent suspension")
    require(chassis.get("wheel_motor")=="DJI_M3508" and chassis.get("reduction")=="planetary","M3508 and planetary reduction must be recorded")
    require(chassis.get("adopted_test_speed_mps")==.5 and chassis.get("automatically_enable_fast") is False,"Keep adopted low-speed baseline; mechanical peak target is not automatic fast authorization")
    gimbal=profile.get("gimbal",{})
    require(gimbal.get("architecture")=="big_yaw_small_yaw" and gimbal.get("big_yaw",{}).get("continuous_rotation") is True,"Continuous big Yaw and limited small Yaw must be represented")
    require(gimbal.get("small_yaw",{}).get("limited_rotation") is True,"Small Yaw has finite limits, distinct from pitch limits")
    require(gimbal.get("pitch_target_degrees")==[-20,25],"Sentry pitch target must not be borrowed from infantry")
    feeder=profile.get("feeder_and_shooter",{})
    require(feeder.get("feed_type")=="central" and feeder.get("magazine_capacity_rounds")==500 and feeder.get("projectile_diameter_mm")==17,"Central feed, 500 rounds and 17mm sentry configuration required")
    require(feeder.get("bench_min_rate_hz")==18 and feeder.get("competition_rate_source")=="referee_heat_and_projectile_budget","Bench rate is not unconditional match firing rate")
    integration=profile.get("integration",{})
    require(integration.get("simultaneous_imu_inputs_to_lio")==1,"Do not feed two unsynchronised IMUs into the existing LIO")
    require(integration.get("required_navigation_lidars")==["primary","auxiliary"],"Both navigation sensor sources must have an explicit health policy")
    require(integration.get("missing_or_uncalibrated_tf_policy")=="refuse_hardware_start","Unmeasured TF must fail closed")
    acceptance=profile.get("acceptance",{})
    require(acceptance.get("integrated_rounds")==3 and acceptance.get("minutes_per_round")==5,"Final report calls for three five-minute rounds")
    require(profile.get("baseline",{}).get("excluded_experiment")=="single_mid_C068","Superseded single-lidar experiment must not become the baseline")
    return errors

def deployment_gaps(profile,full=False):
    gaps=[]
    if not profile.get("integration",{}).get("hardware_deployment_enabled"):gaps.append("hardware deployment is not enabled")
    if not profile.get("integration",{}).get("dual_mid_runtime_implemented") or not profile.get("integration",{}).get("dual_sensor_health_implemented"):
        gaps.append("dual-MID runtime and per-device health integration not implemented")
    for lidar in profile.get("sensors",{}).get("lidars",[]):
        name=lidar.get("id","unknown")
        connection=lidar.get("connection",{})
        if not connection.get("device_id") or not connection.get("ip_address"):gaps.append(name+": measured device ID and network binding missing")
        e=lidar.get("extrinsics",{})
        vectors=[e.get("xyz_m"),e.get("rpy_rad")]
        valid=all(isinstance(v,list) and len(v)==3 and all(isinstance(x,(int,float)) and math.isfinite(x) for x in v) for v in vectors)
        if not e.get("calibrated") or not e.get("parent_frame") or not valid:gaps.append(name+": measured parent/pose calibration missing; zero is not a substitute")
    c=profile.get("chassis",{})
    for name in ["full_vehicle_footprint_m","full_vehicle_mass_kg","wheel_radius_m","gear_ratio","suspension_travel_m"]:
        if c.get(name) is None:gaps.append("chassis: "+name+" not measured")
    compute=profile.get("compute",{})
    if not compute.get("exact_sku") or not compute.get("real_device_built") or not compute.get("real_device_timing_verified"):gaps.append("Orin SKU, native build and timing qualification missing")
    bridge=profile.get("electrical_bridge",{})
    if not bridge.get("controller_board_model") or not bridge.get("transport_protocol") or not bridge.get("command_ack_protocol"):gaps.append("electrical command/feedback/ack protocol missing")
    if full:
        camera=profile.get("sensors",{}).get("camera",{})
        if not camera.get("model") or not camera.get("calibration",{}).get("ready"):gaps.append("industrial camera identity and camera-to-shooter calibration missing")
        g=profile.get("gimbal",{})
        if not g.get("big_yaw",{}).get("zero_calibrated") or not g.get("small_yaw",{}).get("zero_calibrated") or not g.get("small_yaw",{}).get("limits_degrees") or not g.get("actual_pitch_limits_calibrated"):gaps.append("gimbal zeros, small-Yaw limits and measured pitch limits missing")
        if not profile.get("feeder_and_shooter",{}).get("interfaces_implemented") or not profile.get("feeder_and_shooter",{}).get("real_firing_verified"):gaps.append("feeder/shooter feedback and physical qualification missing")
    return gaps

def interface_template(profile):
    errors=inventory_errors(profile)
    if errors:raise ValueError("; ".join(errors))
    return {
        "kind":"NON_DEPLOYABLE_INTERFACE_TEMPLATE",
        "hardware_ready":False,
        "profile_id":profile["profile_id"],
        "driver_policy":{"existing_driver":"livox_ros_driver2","multi_topic":1,
            "frame_binding":"per_device_frame_binding_required; shared launch frame_id is insufficient",
            "extrinsic_owner":"source-time TF; do not transform in SDK and TF twice"},
        "lidars":copy.deepcopy(profile["sensors"]["lidars"]),
        "lio":{"cloud_input":"livox/lidar","imu_input":profile["sensors"]["imu"]["lio_input_topic"],
            "secondary_cloud_is_not_raw_fused":True,"secondary_imu_not_fused":True},
        "auxiliary_obstacle_input":{"cloud_topic":"auxiliary/livox/lidar","frame":"aux_mid360",
            "source_stamp_required":True,"measured_tf_required":True,
            "ground_and_vehicle_self_filter_required":True,"filter_calibration":None},
        "readiness":{"required_lidars":["primary","auxiliary"],
            "failure_action":"cancel active NavigateToPose, confirm terminal and physical stop",
            "deployment_gaps":deployment_gaps(profile,full=True)}}

def audit_legacy_model(path):
    import xml.etree.ElementTree as ET
    root=ET.parse(path).getroot()
    blocks=[x.get("name") for x in root.findall(".//model/xmacro_block")]
    found={"MID-360":blocks.count("livox"),"RPLidar_A2":blocks.count("rplidar_a2")}
    return {"matches_final_lidar_inventory":found=={"MID-360":2,"RPLidar_A2":0},
        "found":found,"expected":{"MID-360":2,"RPLidar_A2":0},
        "legacy_chassis_proxy_present":any(x.get("filename")=="MecanumDrive2" for x in root.findall(".//plugin"))}

def main():
    a=argparse.ArgumentParser()
    a.add_argument("--profile",type=Path,default=DEFAULT_PROFILE)
    a.add_argument("--mode",choices=["plan","navigation-deploy","full-deploy"],default="plan")
    a.add_argument("--source-pdf",type=Path)
    a.add_argument("--output",type=Path)
    a.add_argument("--write-interface-template",type=Path)
    a.add_argument("--audit-legacy-model",type=Path)
    args=a.parse_args()
    profile=yaml.safe_load(args.profile.read_text())
    errors=inventory_errors(profile)
    if args.source_pdf and hashlib.sha256(args.source_pdf.read_bytes()).hexdigest()!=profile["source"]["sha256"]:errors.append("Source PDF SHA256 mismatch")
    gaps=deployment_gaps(profile,full=args.mode=="full-deploy")
    result={"inventory_valid":not errors,"mode":args.mode,"errors":errors,"deployment_ready":not errors and not gaps,"deployment_gaps":gaps,"runtime_changed":False}
    if args.audit_legacy_model:result["legacy_model"]=audit_legacy_model(args.audit_legacy_model)
    if args.write_interface_template:
        if errors:raise SystemExit("; ".join(errors))
        args.write_interface_template.write_text(yaml.safe_dump(interface_template(profile),sort_keys=False,allow_unicode=True))
    if args.output:args.output.write_text(json.dumps(result,indent=2,ensure_ascii=False))
    print(json.dumps(result,indent=2,ensure_ascii=False))
    return 2 if errors or (args.mode!="plan" and gaps) else 0

if __name__=="__main__":raise SystemExit(main())
