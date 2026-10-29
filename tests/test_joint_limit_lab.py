"""Entirely offline tests. No transport/camera objects are constructed."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from joint_limit_lab import schema,trajectories,campaign,simulation,analysis
from drum_planner.experiment.motion_settings import matrix,settings,load as load_profile


@pytest.fixture(scope='module')
def demo():return simulation.demo_campaign('right')


@pytest.mark.parametrize('side',['right','left'])
def test_original_defaults_and_schema(side):
    c=schema.validate(schema.defaults(side))
    assert matrix(c['profile']).tolist()==[[.6]*7,[1.2]*7,[6.]*7]
    assert c['conditions']['supply_voltage_v'] is None


@pytest.mark.parametrize('patch',[
    {'sample_period_s':0},{'repeats':1},{'repeats':True},{'test_levels':[.5,.3]},
    {'test_levels':[1.,2.]},{'test_levels':[float('nan')]},{'displacements_rad':[2.]},
    {'operating_speed_fraction':1.},{'side':'both'},{'surprise':True}])
def test_invalid_settings(patch):
    with pytest.raises((ValueError,TypeError)):schema.validate(dict(schema.defaults('right'),**patch))


@pytest.mark.parametrize('joint',range(1,8))
@pytest.mark.parametrize('direction',[-1,1])
def test_continuous_probe_limits_and_stationary_other_joints(joint,direction):
    anchor=np.linspace(-.2,.3,7);limits=matrix(settings('right'))
    law=trajectories.probe(anchor,joint,direction,.06,limits)
    trajectories.verify(law)
    t=np.linspace(0,law['duration_s'],3001);q,v,a,j=trajectories.evaluate(law,t)
    assert np.all(np.array([abs(v).max(0),abs(a).max(0),abs(j).max(0)])<=limits*(1+1e-9))
    np.testing.assert_allclose(q[[0,-1]],np.tile(anchor,(2,1)),atol=1e-10)
    other=[k for k in range(7) if k!=joint-1]
    np.testing.assert_allclose(q[:,other],np.tile(anchor[other],(len(t),1)),atol=1e-10)
    np.testing.assert_allclose(q[np.argmin(abs(t-law['outward_s']))][joint-1],anchor[joint-1]+direction*.06,atol=1e-8)


def test_speed_scaling_is_power_consistent():
    limits=matrix(settings('left'));a=trajectories.probe([0]*7,2,-1,.05,limits)
    b=trajectories.probe([0]*7,2,-1,.05,limits*2.**np.arange(1,4)[:,None])
    assert b['outward_s']==pytest.approx(a['outward_s']/2,rel=1e-9)


def test_extreme_slow_probe_rejected_before_allocation():
    with pytest.raises(ValueError,match='60-second'):
        trajectories.probe([0]*7,1,1,.04,matrix(settings('right'))*1e-20)


def test_hash_tamper_detected(tmp_path,demo):
    obj=copy.deepcopy(demo);p=tmp_path/'campaign.json';schema.write(p,obj)
    assert campaign.load(p)['sha256']==demo['sha256']
    obj['trials'][0]['law']['limits'][0][0]*=2;schema.write(p,obj)
    with pytest.raises(ValueError,match='checksum'):campaign.load(p)


@pytest.mark.parametrize('failure,reason',[
    ('tracking','tracking_error'),('stale','repeated_stale_feedback'),('temperature','temperature'),
    ('current','current_headroom'),('motor_fault','motor_fault'),('slow','tracking_error'),('command_gap','command_gap')])
def test_faults_never_pass(demo,failure,reason):
    trial=demo['trials'][0];meta,data=simulation.trace(demo,trial,failure=failure)
    result=analysis.score(demo,trial,meta,data)
    assert not result['passed'] and reason in result['reasons']


def test_clean_trace_and_derivative_noise_not_used_as_limits(demo):
    trial=demo['trials'][0];meta,data=simulation.trace(demo,trial)
    r=analysis.score(demo,trial,meta,data)
    assert r['passed'] and len(r['measured_derivatives_diagnostic_only'])==2
    assert r['commanded_peaks']==trial['law']['peaks']
    assert r['requested_limit_excitation_fraction'][0]<.9  # short stroke cannot prove the speed ceiling


def test_stale_current_not_counted_as_load_evidence(demo):
    trial=demo['trials'][0];m,d=simulation.trace(demo,trial);d['current_age_s'][-20:,2]=1.
    r=analysis.score(demo,trial,m,d)
    assert not r['passed'] and 'stale_current_feedback' in r['reasons']


@pytest.mark.parametrize('joint',[1,3,5,7])
@pytest.mark.parametrize('side',['right','left'])
def test_recorded_frame_decoder_matches_installed_conventions(joint,side):
    import struct
    from safe_zone.encoder import FRAME,EFF,encoder_to_joint
    from joint_limit_lab.telemetry import decode,VELOCITY_SCALES,TORQUE_SCALES
    raw=(40000,45000,20000,315)
    frame=FRAME.pack(EFF|2<<24|2<<22|joint<<8|0xfd,8,struct.pack('>4H',*raw))
    x=decode(frame,side)
    assert x['q_rad']==encoder_to_joint(side,joint,raw[0]/65535*25.14-12.57)
    assert x['velocity_rad_s']==-(raw[1]/65535*2-1)*VELOCITY_SCALES[joint-1]
    assert x['torque_nm']==-(raw[2]/65535*2-1)*TORQUE_SCALES[joint-1]
    assert x['mode']==2 and x['fault_bits']==0 and x['temperature_c']==31.5
    p=FRAME.pack(EFF|17<<24|joint<<8|0xfd,8,struct.pack('<H2xf',0x701A,-1.25))
    assert decode(p,side)['value']==-1.25


@pytest.mark.parametrize('mutation',['nan','times','arm','law','truncated','bounds','mode','sequence'])
def test_bad_evidence_cannot_pass(demo,mutation):
    trial=demo['trials'][0];meta,data=simulation.trace(demo,trial);c=copy.deepcopy(demo)
    if mutation=='nan':data['q_rad'][3,1]=np.nan
    if mutation=='times':data['time_s'][3]=data['time_s'][2]
    if mutation=='arm':meta['side']='left'
    if mutation=='law':data['command_q_rad'][3,1]+=.02
    if mutation=='truncated':data={k:v[:-50] for k,v in data.items()}
    if mutation=='bounds':c['position_bounds_rad']=[[-.01,.01]]*7
    if mutation=='mode':data['mode'][20,0]=0
    if mutation=='sequence':data['feedback_sequence'][10,0]=-1
    try:r=analysis.score(c,trial,meta,data)
    except ValueError:return
    assert not r['passed']


def test_missing_current_is_not_silently_zero(tmp_path,demo):
    meta,data=simulation.trace(demo,demo['trials'][0]);data.pop('current_a');p=tmp_path/'bad.npz'
    simulation.save_trace(p,meta,data)
    with pytest.raises(ValueError,match='fields'):analysis.load_trace(p)


def test_npz_roundtrip_exact(tmp_path,demo):
    meta,data=simulation.trace(demo,demo['trials'][0]);p=tmp_path/'trace.npz';simulation.save_trace(p,meta,data)
    m,d=analysis.load_trace(p);assert m==meta
    for key in data:np.testing.assert_array_equal(d[key],data[key])
    with pytest.raises(FileExistsError):simulation.save_trace(p,meta,data)


@pytest.fixture(scope='module')
def all_traces(tmp_path_factory,demo):
    folder=tmp_path_factory.mktemp('traces')/'records'
    simulation.simulate(demo,folder)
    return folder


def test_complete_synthetic_export_is_explicit(tmp_path,demo,all_traces):
    report=analysis.analyze(demo,all_traces,tmp_path/'no_export')
    assert report['complete_probe_coverage'] and report['synthetic'] and report['profile'] is None
    report=analysis.analyze(demo,all_traces,tmp_path/'export',allow_synthetic=True)
    path=Path(report['profile']);p=load_profile(path,'right')
    assert path.name.startswith('SYNTHETIC_NOT_FOR_HARDWARE_') and not report['hardware_limits_known']
    np.testing.assert_allclose(matrix(p),np.array(report['observed_commanded_envelope'])*.8**np.arange(1,4)[:,None])
    assert not schema.read(path.with_suffix('.evidence.json'))['combined_route_physically_validated']


def test_missing_and_duplicate_evidence_blocks_export(tmp_path,demo,all_traces):
    source=next(all_traces.glob('*.npz'));folder=tmp_path/'traces';folder.mkdir()
    (folder/'a.npz').write_bytes(source.read_bytes());(folder/'b.npz').write_bytes(source.read_bytes())
    r=analysis.analyze(demo,folder,tmp_path/'result',allow_synthetic=True)
    assert r['profile'] is None and r['missing_coverage'] and r['invalid_records']


def test_failure_stops_escalation(demo,tmp_path):
    r=simulation.simulate(demo,tmp_path/'traces',failure='tracking',fail_level=.75)
    assert r['stopped_groups'] and r['not_run_after_failure']>0
    assert not any('-l2-' in t['id'] for t in r['trials'])
    a=analysis.analyze(demo,tmp_path/'traces',tmp_path/'report',allow_synthetic=True)
    assert all(g['highest_complete_level']==.5 for g in a['groups'])


def test_mixed_synthetic_physical_never_exports(tmp_path,demo,all_traces):
    folder=tmp_path/'records';folder.mkdir()
    for i,p in enumerate(all_traces.glob('*.npz')):
        m,d=analysis.load_trace(p)
        if i==0:m['origin']='physical_recording'
        simulation.save_trace(folder/p.name,m,d)
    r=analysis.analyze(demo,folder,tmp_path/'report',allow_synthetic=True)
    assert r['profile'] is None and len(r['evidence_origins'])==2


def test_empty_evidence_is_unknown_not_defaults(tmp_path,demo):
    (tmp_path/'empty').mkdir();r=analysis.analyze(demo,tmp_path/'empty',tmp_path/'report')
    assert not r['hardware_limits_known'] and r['profile'] is None and len(r['missing_coverage'])==28


def test_cli_has_no_accidental_hardware_path():
    from joint_limit_lab.cli import parser
    with pytest.raises(SystemExit):parser().parse_args(['--hardware'])
    code='from drum_planner.experiment.config import no_can; no_can(); import socket; socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)'
    r=subprocess.run([sys.executable,'-c',code],capture_output=True,text=True)
    assert r.returncode and 'forbids CAN' in r.stderr


def test_zero_exit_without_receipt_is_not_success(tmp_path,monkeypatch):
    from types import SimpleNamespace
    from joint_limit_lab.cli import main
    from drum_planner.experiment import runtime
    class Finished:
        process=SimpleNamespace(returncode=0);log_path=tmp_path/'worker.log'
        def __init__(self,*a,**kw):pass
        def poll(self):return 0
        def close(self):pass
    monkeypatch.setattr(runtime,'Job',Finished)
    assert main(['demo','--right','--output',str(tmp_path/'no_receipt')])==1


def test_existing_programs_and_data_untouched():
    root=Path(__file__).resolve().parents[1]
    before=schema.read(root/'diagnostics/joint_limit_testing_20261025/before_hashes.json')
    allowed={'.gitignore','config/git_retained_artifacts.json'}
    for name,digest in before.items():
        if name not in allowed:assert schema.file_hash(root/name)==digest,name
