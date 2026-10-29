from pathlib import Path
import importlib.util
import json
import tempfile
import unittest
spec=importlib.util.spec_from_file_location("capture",Path(__file__).with_name("timing_capture.py"))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
entry_spec=importlib.util.spec_from_file_location("capture_entry",Path(__file__).resolve().parents[1]/"run_timing_capture.py")
entry=importlib.util.module_from_spec(entry_spec);entry_spec.loader.exec_module(entry)
class ClockDivergenceContract(unittest.TestCase):
 def row(self,native=10.,ros=10.):
  return {"steady":5.,"streams":{k:{"count":30,"stamp":v,"received":4.99} for k,v in [("native_clock",native),("ros_clock",ros)]}}
 def test_synchronized_clock_does_not_trigger(self):
  self.assertIsNone(m.clock_divergence_for(self.row()))
 def test_progressing_but_delayed_ros_clock_triggers(self):
  v=m.clock_divergence_for(self.row(10.,9.))
  self.assertEqual(v["reason"],"clock_source_divergence");self.assertEqual(v["delta_s"],1.)
  self.assertEqual(v["causality"],"not_determined")
 def test_separates_upstream_native_clock_difference(self):
  r=self.row(10.,9.);r["streams"]["native_global_clock"]={"count":30,"stamp":9.,"received":4.99}
  self.assertEqual(m.clock_divergence_for(r)["stage"],"native_world_to_global")
 def test_native_global_reference_is_explicit(self):
  r=self.row(10.,9.);r["streams"]["native_global_clock"]={"count":30,"stamp":10.,"received":4.99}
  self.assertEqual(m.clock_divergence_for(r)["reference"],"native_global_clock")
 def test_shutdown_or_observer_missing_receipt_not_causal_event(self):
  r=self.row(10.,9.);r["streams"]["ros_clock"]["received"]=4.
  self.assertIsNone(m.clock_divergence_for(r))
 def test_missing_and_nan_not_evidence(self):
  self.assertIsNone(m.clock_divergence_for({"steady":5.,"streams":{}}))
  self.assertIsNone(m.clock_divergence_for(self.row(float("nan"),9.)))
 def test_clock_window_survives_first_expected_tf_failure(self):
  with tempfile.TemporaryDirectory() as p:
   a=m.Window(p,filename="tf.jsonl");b=m.Window(p,filename="clock.jsonl")
   for i in range(200):
    row=self.row(10+i*.01,10+i*.01-(.3 if i>=100 else 0));row["steady"]=i*.05
    for v in row["streams"].values():v["received"]=row["steady"]-.01
    a.accept(row,{"reason":"PoseFresh_FAILURE"} if i==20 else None)
    b.accept(row,m.clock_divergence_for(row))
   self.assertTrue(a.finish()["post_complete"]);self.assertTrue(b.finish()["post_complete"])
   self.assertTrue((Path(p)/"clock.jsonl").exists())

class CaptureContract(unittest.TestCase):
 def test_guarded_entry_preserves_child_output_arguments(self):
  argv=["entry","--output","capture","--","python3","child.py","--output","child-output"]
  self.assertEqual(entry.normalize_output_arguments(argv),["entry","--record-dir","capture","--","python3","child.py","--output","child-output"])
  self.assertEqual(argv[1],"--output")
 def test_guarded_entry_accepts_equals_form_without_touching_child(self):
  self.assertEqual(entry.normalize_output_arguments(["entry","--output=capture","--","child","--output=x"]),["entry","--record-dir=capture","--","child","--output=x"])

 def test_actual_bounded_pre_post_history(self):
  with tempfile.TemporaryDirectory() as p:
   w=m.Window(p)
   for i in range(161):
    t=i*.05;w.accept({"steady":t,"wall":100+t,"payload":i},
      {"reason":"injected"} if i==100 else None)
   s=w.finish();rows=[json.loads(x) for x in (Path(p)/"event.jsonl").read_text().splitlines()]
   self.assertGreaterEqual(s["pre_available_s"],2.95)
   self.assertTrue(s["post_complete"])
   self.assertGreaterEqual(rows[0]["steady"],2.)
   self.assertLessEqual(rows[-1]["steady"],7.05)
 def test_no_event_does_not_write_full_stream(self):
  with tempfile.TemporaryDirectory() as p:
   w=m.Window(p,ring_bytes=300)
   for i in range(10000):w.accept({"steady":i*.01,"value":"a"*40})
   self.assertLessEqual(w.ring_bytes,300);self.assertFalse((Path(p)/"event.jsonl").exists())
   self.assertIsNone(w.finish()["trigger"])
 def test_disk_byte_limit_is_explicitly_incomplete(self):
  with tempfile.TemporaryDirectory() as p:
   w=m.Window(p,max_bytes=250)
   for i in range(100):w.accept({"steady":i*.05,"data":"x"*80},{"reason":"injected"} if i==1 else None)
   s=w.finish();self.assertTrue(s["truncated"]);self.assertFalse(s["post_complete"])
   self.assertLessEqual((Path(p)/"event.jsonl").stat().st_size,250)
 def test_truncated_child_exit_not_full_post_window(self):
  with tempfile.TemporaryDirectory() as p:
   w=m.Window(p);w.accept({"steady":1.},{"reason":"injected"});w.accept({"steady":1.1})
   self.assertFalse(w.finish()["post_complete"])
 def test_missing_streams_are_not_a_diagnosed_stall(self):
  self.assertIsNone(m.trigger_for({"steady":10.,"streams":{}} ,True))
 def test_observer_gap_is_recorded_as_missing_coverage(self):
  streams={k:{"count":20,"progress":10.} for k in m.REQUIRED}
  r=m.trigger_for({"steady":10.1,"streams":streams,"loop_gap":.3},True)
  self.assertEqual(r["reason"],"observer_gap");self.assertEqual(r["causality"],"uncovered_observation")
 def test_slow_clock_frozen_tf_is_observed_without_relaxing_a_guard(self):
  streams={k:{"count":20,"progress":10.7} for k in m.REQUIRED}
  streams["tf:map->odom"]["progress"]=10.
  r=m.trigger_for({"steady":10.7,"streams":streams},True)
  self.assertEqual(set(r["progress_age_s"]),{"tf:map->odom"})
  self.assertEqual(r["causality"],"not_determined")
 def test_rosout_failure_triggers_even_without_complete_headers(self):
  r=m.trigger_for({"steady":10.,"failures":[{"message":"PoseFresh FAILURE: stamp_stalled"}]},False)
  self.assertEqual(r["reason"],"PoseFresh_FAILURE")
 def test_invalid_nan_is_not_serialized_as_evidence(self):
  with tempfile.TemporaryDirectory() as p:
   w=m.Window(p);w.accept({"steady":float("nan")},{"reason":"injected"})
   self.assertEqual(w.finish()["invalid_records"],1)
 def test_closed_goal_does_not_arm_shutdown_but_next_goal_does(self):
  with tempfile.TemporaryDirectory() as p:
   root=Path(p);goal=root/"first";goal.mkdir()
   (goal/"goal-ownership.json").write_text('{"goal_uuid":"00000000000000000000000000000001"}')
   self.assertTrue(m.active_owned_goal(root))
   (goal/"stop-state.json").write_text('{"original_1s_stop_passed":true}')
   self.assertFalse(m.active_owned_goal(root))
   second=root/"second";second.mkdir();(second/"goal-ownership.json").write_text("{}")
   self.assertTrue(m.active_owned_goal(root))
 def test_resources_retain_per_thread_identities(self):
  r=m.resource_snapshot()
  self.assertEqual(r["type"],"resource");self.assertIn("clipped",r)
  for process in r["processes"]:
   for thread in process["threads"]:
    self.assertEqual(len(thread["schedstat"]),3)
    self.assertGreater(thread["start_ticks"],0)
 def test_rejected_occupied_goal_is_terminal_without_stop_snapshot(self):
  with tempfile.TemporaryDirectory() as p:
   root=Path(p);goal=root/'occupied';goal.mkdir()
   identifier='00000000000000000000000000000001'
   (goal/'goal-ownership.json').write_text(json.dumps({'goal_uuid':identifier}))
   trace=goal/'verify.jsonl'
   trace.write_text(json.dumps({'event':'goal_accepted','mode':'nav','goal_uuid':identifier})+'\n')
   self.assertTrue(m.active_owned_goal(root))
   with trace.open('a') as out:out.write(json.dumps({'event':'invalid_result','mode':'nav','status':6})+'\n')
   self.assertFalse(m.active_owned_goal(root))
   second=root/'next';second.mkdir();(second/'goal-ownership.json').write_text('{}')
   self.assertTrue(m.active_owned_goal(root))
 def test_unrelated_terminal_cannot_disarm_owned_goal(self):
  with tempfile.TemporaryDirectory() as p:
   goal=Path(p);(goal/'goal-ownership.json').write_text('{"goal_uuid":"owned"}')
   (goal/'verify.jsonl').write_text('{"event":"goal_accepted","mode":"nav","goal_uuid":"other"}\n{"event":"invalid_result","mode":"nav","status":6}\n')
   self.assertTrue(m.active_owned_goal(goal))
 def test_partial_terminal_record_does_not_disarm(self):
  with tempfile.TemporaryDirectory() as p:
   goal=Path(p);(goal/'goal-ownership.json').write_text('{"goal_uuid":"owned"}')
   (goal/'verify.jsonl').write_text('{"event":"goal_accepted","mode":"nav","goal_uuid":"owned"}\n{"event":"invalid_result"')
   self.assertTrue(m.active_owned_goal(goal))
 def test_real_occupied_verifier_fixture_disarms(self):
  with tempfile.TemporaryDirectory() as p:
   goal=Path(p);identifier='30cca3e0eb2d46d59b93f4db06a7953d'
   (goal/'goal-ownership.json').write_text(json.dumps({'goal_uuid':identifier}))
   (goal/'verify.jsonl').write_text('\n'.join(json.dumps(x) for x in [
    {'event':'invalid_goal','mode':'nav','xy':[9.175,1.025],'occupancy':100},
    {'event':'goal_accepted','mode':'nav','goal_uuid':identifier},
    {'event':'invalid_result','mode':'nav','status':6,'path_points':0,'ground_truth_moved':.00002599,'nonzero_commands':0},
    {'event':'PASS','mode':'nav','scenario':'invalid'}])+'\n')
   self.assertFalse(m.active_owned_goal(goal))
 def test_actual_gazebo_process_title_is_sampled_instead_of_launcher(self):
  self.assertTrue(m.observed_process("ign gazebo -r -s --headless-rendering /project/world.sdf   "))
  self.assertFalse(m.observed_process("/bin/sh -c ruby /usr/bin/ign gazebo -r -s"))
  self.assertFalse(m.observed_process("python3 capture.py --observer /project/timing_chain_observer --identity files.json"))
  self.assertTrue(m.observed_process("/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --ros-args"))
if __name__=="__main__":unittest.main(verbosity=2)
