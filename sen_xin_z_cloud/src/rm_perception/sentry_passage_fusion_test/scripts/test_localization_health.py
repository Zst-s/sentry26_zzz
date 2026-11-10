#!/usr/bin/env python3
"""Health watchdog and formal v4 history during transient localization failure."""
import json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
from localization_health import LocalizationHealth
from continuous_passage_memory import ContinuousPassageMemory
from global_passage_memory import MapContext
from live_memory_shadow import LiveMemoryShadow

def diagnostic(state='TRACKING',seq=1,session='lio-A'):
 m=DiagnosticArray();m.header.stamp.sec=10;s=DiagnosticStatus();s.name='pointlio/localization';s.hardware_id=session
 s.level=b'\x00' if state=='TRACKING' else b'\x01'
 s.values=[KeyValue(key=k,value=v) for k,v in {'state':state,'sequence':str(seq),'generation':'0','last_valid_stamp':'10'}.items()];m.status=[s];return m

class HealthTests(unittest.TestCase):
 def test_replay_does_not_refresh_and_identity_change_is_latched(self):
  h=LocalizationHealth();h.update(diagnostic(),0.);self.assertTrue(h.ready(.1))
  h.update(diagnostic(),.2);self.assertFalse(h.ready(.3))
  h.update(diagnostic(seq=2),.31);self.assertTrue(h.ready(.32))
  h.update(diagnostic(seq=3,session='lio-B'),.33);self.assertFalse(h.ready(.34))
  h.update(diagnostic(seq=4),.35);self.assertFalse(h.ready(.36))
 def test_recovered_status_carries_missed_fault_generation(self):
  h=LocalizationHealth();h.update(diagnostic(),0.)
  m=diagnostic(seq=2)
  next(v for v in m.status[0].values if v.key=='generation').value='1'
  h.update(m,.1);self.assertTrue(h.ready(.11));self.assertEqual(h.generation,1)
 def test_new_heartbeat_cannot_make_old_pose_fresh(self):
  h=LocalizationHealth();m=diagnostic();m.header.stamp.sec=11
  h.update(m,0.);self.assertFalse(h.ready(.01))
 def test_cancel_transport_exception_keeps_old_goal_blocked(self):
  from localization_health import NavigationRecovery
  client=SimpleNamespace(service_is_ready=lambda:True,
      call_async=lambda request:(_ for _ in ()).throw(RuntimeError('injected transport failure')))
  node=SimpleNamespace(create_client=lambda *args:client,
      get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10000000000)))
  gate=NavigationRecovery(node);gate.health.update(diagnostic());gate.was_ready=True
  key=bytes([7]*16);gate.retired={key};gate.active={key:0};gate.seen_status=True
  self.assertFalse(gate.tick());self.assertEqual(gate.retired,{key})
  self.assertIn('transport failed',gate.reason)
 def test_malformed_does_not_authorize(self):
  h=LocalizationHealth();h.update(diagnostic(),0.)
  m=diagnostic(seq=2);m.status[0].values.append(m.status[0].values[0]);h.update(m,.1)
  self.assertFalse(h.ready(.11))
 def test_v4_history_survives_pause_recovers_and_rejects_changed_session(self):
  context=MapContext('mapping-health-test','map',10,10,.05,0,0,0,0,0);grid=[100]*100
  observation=dict(frame='odom',measured_at=10.,resolution=.05,
      columns=[(.025+x*.05,.025+y*.05,-.2,.2) for y in range(4) for x in range(4)])
  with tempfile.TemporaryDirectory() as d:
   m=ContinuousPassageMemory();m.add_structure(context=context,grid=grid,observation=observation,cells=(),map_transform=(0,0,0,0),now=10.1)
   p=Path(d)/'history.json';m.save(p);saved=p.read_bytes();expected=m.planning_cells(context,grid);groups={k:set(v) for k,v in m.groups.items()}
   h=LocalizationHealth();h.update(diagnostic('DEGRADED'))
   masks=[];a=SimpleNamespace(require_health=True,localization_health=h,health_was_ready=True,health_boundary=0.,health_generation=0.,
       get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10200000000)),
       memory=m,grid=SimpleNamespace(data=grid),lookup=lambda _:None,map_session_id=context.epoch,runtime=True,
       restored=True,pending=None,pending_structure=object(),pending_terrain=object(),record_path=str(p),
       reason='',accept_structure=lambda *args:None,publish_mask=lambda c:masks.append(c),publish_status=lambda *args:None)
   LiveMemoryShadow.tick(a);self.assertIsNone(masks[-1]);self.assertEqual(m.state,'KNOWN_PASSAGE')
   self.assertEqual(m.groups,groups);self.assertEqual(p.read_bytes(),saved);self.assertIsNone(a.pending_structure)
   h.update(diagnostic(seq=2))
   with patch('live_memory_shadow.map_context',return_value=context):LiveMemoryShadow.tick(a)
   self.assertEqual(masks[-1],context);self.assertEqual(m.planning_cells(context,grid),expected)
   # A delayed consumer callback stops motion without revoking associated history.
   h.received -= .6
   self.assertFalse(h.ready())
   with patch('live_memory_shadow.map_context',return_value=context):LiveMemoryShadow.tick(a)
   self.assertEqual(masks[-1],context);self.assertEqual(m.groups,groups)
   h.update(diagnostic('NEEDS_RESET',seq=3));LiveMemoryShadow.tick(a)
   self.assertIsNone(masks[-1]);self.assertEqual(p.read_bytes(),saved)
   h.update(diagnostic(seq=4,session='lio-B'));LiveMemoryShadow.tick(a)
   self.assertIsNone(masks[-1]);self.assertEqual(m.groups,groups)
if __name__=='__main__':unittest.main()
