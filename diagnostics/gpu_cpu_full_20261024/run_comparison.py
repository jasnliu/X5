"""Cold-session, actual experiment-worker comparison; no production edits/hardware.

Each backend starts with its own empty persistent and run-local caches. Normal
within-session reuse between the five methods remains, exactly as in the GUI.
"""
import hashlib,importlib.util,json,os,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];BASE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT))
from drum_planner.common import atomic_json,digest,read_state,endpoint
from drum_planner.experiment import config,checkpoints
from drum_planner.experiment.runtime import Job,PYTHON
from drum_planner.timing import core


def source_hashes():
    paths=[]
    for root in ('drum_planner','destination_capture','model','destinations'):
        for p in (ROOT/root).rglob('*'):
            if p.is_file() and 'build' not in p.parts and '__pycache__' not in p.parts and p.suffix in ('.py','.cpp','.h','.hpp','.comp','.glsl','.json','.sh','.urdf','.srdf'):
                paths.append(p)
    return {str(p.relative_to(ROOT)):digest(p) for p in sorted(paths)}


def prepare():
    if (BASE/'protocol.json').exists():raise ValueError('Use a new benchmark folder; do not reuse cold caches')
    before=source_hashes();atomic_json(BASE/'source_before.json',before)
    current=(ROOT/'drum_planner/experiment/worker.py').read_text()
    start=current.index("    # Experiment-only routing. Hardware never calls this worker's plan().\n")
    end=current.index('    from .algorithms import run\n',start)
    old=current[:start]+"    if len(context.get('checkpoints',[]))>1:\n        from .checkpoints import plan_sequence\n        return plan_sequence(context,folder,method,params,seconds,plan,parallel_timing=True)\n"+current[end:]
    expected=json.loads((ROOT/'diagnostics/gpu_timing_20261024/before_hashes.json').read_text())
    assert hashlib.sha256(old.encode()).hexdigest()==expected['drum_planner/experiment/worker.py']
    (BASE/'cpu_worker_before_gpu.py').write_text(old)
    for name in ('adaptive.py','core.py','integration.py','smooth_v2.py','selected.json','selected_v2.json','pipeline.py','math_cache.py'):
        key='drum_planner/timing/'+name;assert digest(ROOT/key)==expected[key],key
    context=json.loads((ROOT/'planner_runs/20261024T002242Z-experiment-c440af/context.json').read_text())
    assert digest(context['state'])==context['state_sha256'] and digest(context['scene'])==context['scene_sha256']
    inputs=BASE/'inputs';inputs.mkdir(exist_ok=True)
    context.update(checkpoints.freeze([ROOT/'destinations/right/destinationR1.json',ROOT/'destinations/right/destinationR4.json'],'right',inputs))
    context.pop('pool_root',None);context.pop('timing_backend',None);context.pop('experiment_gpu_timing',None)
    atomic_json(inputs/'context.json',context);atomic_json(inputs/'settings.json',config.settings())
    paths=[Path(context[k]) for k in ('state','scene','calibration','zone_path')]
    paths += [Path(b['goal_recording']) for b in context['checkpoints']]
    scene=json.loads(Path(context['scene']).read_text())
    paths += [(Path(context['scene']).parent/scene[k]).resolve()
              for k in ('points_file','free_depth_file') if scene.get(k)]
    state=read_state(context['state'])
    protocol=dict(schema='x5-cold-gpu-vs-original-cpu-v1',backend_order=['gpu','cpu'],method_order=list(config.METHODS),
        route='saved rest -> R1 -> R4 -> independently planned original rest',moving_side='right',
        start=state['right_rad'],checkpoints=[endpoint(Path(b['goal_recording']),side='right').tolist() for b in context['checkpoints']],
        budget_per_leg_s=120,timing_workers=2,memory_cap_bytes=3*1024**3,
        cache_policy='Each backend starts with empty, isolated run-local and persistent caches; normal reuse WITHIN its five-method session retained; no prior results accessible',
        includes=['fresh path generation/ranking','CPU-reference qualification in GPU version','timing','continuous/dense collision/zone checks','checkpoint composition','worker startup'],
        excludes=['GUI rendering/startup','scene acquisition/preparation before destination selection','dependency compilation/install','physical playback'],
        repetitions_per_backend=1,original_cpu_worker_byte_identical=True,original_cpu_worker_sha256=expected['drum_planner/experiment/worker.py'],
        inputs_sha256={str(p):digest(p) for p in paths},motor_commands=0,camera_access=False)
    atomic_json(BASE/'protocol.json',protocol)
    return protocol,context


def result_row(folder):
    path=folder/'plan.json'
    if not path.exists():return dict(status='missing_plan',legs=[])
    plan=json.loads(path.read_text());row=dict(status=plan['status'],error=plan.get('error'),legs=[])
    assert plan.get('motion_authorized') is False and plan.get('motor_commands')==0
    if plan['status']!='planned':return row
    sequence=plan['checkpoint_sequence'];assert len(sequence['segments'])==2
    for name in ('checkpoint_1','checkpoint_2','return_to_start'):
        p=json.loads((folder/name/'plan.json').read_text());req=json.loads((folder/name/'backend_request.json').read_text())
        assert p['status']=='planned' and p['final_curve_check']['samples_checked']>1
        assert p['outward'][0]['positions_rad']==req['start'] and p['outward'][-1]['positions_rad']==req['goal']
        info=p['experiment']['retiming'];report=p['experiment']['report']
        row['legs'].append(dict(leg=name,geometry_sha256=core.identity(dict(path_waypoints=p['path_waypoints'],timing=p['source_timing'])),
            source_timing_sha256=hashlib.sha256(json.dumps(p['source_timing'],sort_keys=True).encode()).hexdigest(),
            selected=report['selected'],pool_key=report['pool_key'],pool_reused=report['pool_reused'],
            start=req['start'],goal=req['goal'],movement_s=p['duration_outward_s'],
            backend=info.get('actual_backend','original_cpu'),retiming=info,
            exact_curve_sha256=hashlib.sha256(json.dumps(p['timing'].get('path'),sort_keys=True).encode()).hexdigest()))
    row['preview_cycle_s']=plan['cycle'][-1]['time_s'];row['reported_worker_elapsed_s']=plan['experiment']['elapsed_s']
    return row


def main():
    config.no_can();protocol,context=prepare()
    from drum_planner.timing.vulkan.device import probe
    atomic_json(BASE/'gpu_device.json',probe())
    all_results={}
    for backend in protocol['backend_order']:
        folder=BASE/backend;folder.mkdir();cache=folder/'persistent_cache';cache.mkdir();pool=folder/'shared_pools';pool.mkdir()
        assert not list(cache.iterdir()) and not list(pool.iterdir())
        os.environ['X5_PLANNER_CACHE_DIR']=str(cache)
        local=dict(context,pool_root=str(pool));atomic_json(folder/'context.json',local)
        summary=dict(backend=backend,methods=[],status='running',cache_initially_empty=True,
            cache_root=str(cache),pool_root=str(pool),utc_start=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),load_start=os.getloadavg())
        atomic_json(folder/'summary.json',summary);began=time.monotonic()
        for method in protocol['method_order']:
            target=folder/'plans'/method;target.mkdir(parents=True)
            atomic_json(target/'context.json',local);atomic_json(target/'settings.json',config.settings())
            command=[str(PYTHON),'-m','drum_planner.experiment.worker'] if backend=='gpu' else [str(PYTHON),str(BASE/'cpu_entry.py')]
            command+=['plan','--folder',str(target),'--context',str(target/'context.json'),'--method',method,
                      '--settings',str(target/'settings.json'),'--budget','120']
            atomic_json(target/'command.json',dict(argv=command,cache_root=str(cache)))
            print(f'\n=== COLD {backend.upper()} / {method} / ALL 3 LEGS ===',flush=True)
            start=time.monotonic();job=Job(command,target,timeout=2160)
            try:
                while job.poll() is None:time.sleep(.05)
                code=job.poll()
            finally:job.close()
            elapsed=time.monotonic()-start
            row=result_row(target);row.update(method=method,wall_s=elapsed,exit_code=code,plan=str(target/'plan.json'),containment=job.limits)
            summary['methods'].append(row);summary['queue_wall_s']=time.monotonic()-began
            atomic_json(folder/'summary.json',summary)
            print(f'=== FINISHED {backend.upper()} / {method}: {elapsed:.3f}s, status={row["status"]}, exit={code} ===',flush=True)
        summary.update(status='complete' if all(r['status']=='planned' and r['exit_code']==0 for r in summary['methods']) else 'some_methods_failed',
            queue_wall_s=time.monotonic()-began,utc_end=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),load_end=os.getloadavg())
        summary['summed_method_wall_s']=sum(r['wall_s'] for r in summary['methods'])
        atomic_json(folder/'summary.json',summary);all_results[backend]=summary
        print(f'=== {backend.upper()} TOTAL: {summary["queue_wall_s"]:.3f}s; {summary["status"]} ===',flush=True)
    comparison=[]
    for g,c in zip(all_results['gpu']['methods'],all_results['cpu']['methods']):
        rows=[]
        for gl,cl in zip(g['legs'],c['legs']):
            rows.append(dict(leg=gl['leg'],same_geometry=gl['geometry_sha256']==cl['geometry_sha256'],
                same_source_timing=gl['source_timing_sha256']==cl['source_timing_sha256'],same_selection=gl['selected']==cl['selected'],
                same_start_goal=gl['start']==cl['start'] and gl['goal']==cl['goal'],movement_ratio=gl['movement_s']/cl['movement_s']))
        comparison.append(dict(method=g['method'],gpu_s=g['wall_s'],cpu_s=c['wall_s'],difference_s=g['wall_s']-c['wall_s'],legs=rows))
    for name,sha in protocol['inputs_sha256'].items():assert digest(name)==sha,name
    before=json.loads((BASE/'source_before.json').read_text());after=source_hashes();assert before==after,'Existing program source changed during benchmark'
    totals={k:v['queue_wall_s'] for k,v in all_results.items()}
    atomic_json(BASE/'comparison.json',dict(totals_s=totals,difference_s=totals['gpu']-totals['cpu'],
        percent_difference=100*(totals['gpu']/totals['cpu']-1),methods=comparison,existing_source_unchanged=True,
        input_hashes_unchanged=True,status={k:v['status'] for k,v in all_results.items()},motor_commands=0,camera_access=False))
    print('FULL COMPARISON COMPLETE',totals,flush=True)

if __name__=='__main__':main()
