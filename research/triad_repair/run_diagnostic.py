#!/usr/bin/env python3
"""Isolated SIMULATION diagnostic; never installs or overwrites old evidence.
Requires an already-built repair controller. This is not an H1 campaign runner.
"""
import argparse, hashlib, json, os, pathlib, subprocess, time
import yaml
P = pathlib.Path
root = P(__file__).resolve().parents[2]
p = argparse.ArgumentParser()
p.add_argument('output', type=P)
p.add_argument('--build', type=P, default=P('/tmp/triad-scientific-repair-build'))
p.add_argument('--scenario', choices=['longitudinal','lateral-low','near-ground','diagonal'], default='longitudinal')
p.add_argument('--seconds', type=float, default=35, help='nominal wall-time allowance; wrapper adds 60 s startup/computation grace')
p.add_argument('--override', type=P)
a = p.parse_args()
out = a.output.resolve(); out.mkdir(parents=True, exist_ok=False)
# mc_rtc always reads the per-controller user fragment. Refuse if it could
# override this isolated configuration; never modify it or repurpose HOME.
user = P.home()/'.config/mc_rtc/controllers'
assert not any((user/('HandoverInterceptionController'+ext)).exists() for ext in ['.yaml','.yml','.conf'])
module = out/'modules'; (module/'etc').mkdir(parents=True)
for name in ['HandoverInterceptionController_controller.so','libHandoverInterceptionController.so']:
    (module/name).symlink_to(a.build.resolve()/'src'/name)
c = yaml.safe_load((a.build/'etc/HandoverInterceptionController.yaml').read_text())
c['StatesLibraries'] = ['/home/harry/mc_rtc_ws/install/lib/mc_controller/fsm/states',str(a.build.resolve()/'src/states')]
c['robots']['call_object']['module'] = ['object',str(root/'call_object_description'),'call_object']
poses={'longitudinal':([.92,0,.55],[-.08,0,0]),'lateral-low':([.55,-.56,.15],[0,.08,0]),'near-ground':([.25,.62,.15],[0,-.08,0]),'diagonal':([.90,0,.30],[-.0565685,0,.0565685])}
pos,vel=poses[a.scenario]
c['robots']['call_object']['init_pos']['translation']=pos
c['object']['translation']=pos
c['movingObject']['simulatedLinearVelocity']=vel
c['movingObject']['giverTruthModel']='independent_scripted'
c['receiverArchitecture']='v2_receding'
v=c['configs']['HandoverInterceptionController_ReceiverV2']
v['supervisorMode']='control_aware'
v.setdefault('controlAware',{}).update(variant='predictive',graspFamily='receiving',admissionRequiresCurrentAuthority=False)
def merge(x,y):
    for k,v in y.items():
        if isinstance(v,dict) and isinstance(x.get(k),dict): merge(x[k],v)
        else: x[k]=v
if a.override: merge(c,yaml.safe_load(a.override.read_text()))
(module/'etc/HandoverInterceptionController.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
g={'MainRobot':['env','/home/harry/mc_rtc_ws/Sandbox/kinova_gen3_2f85_mcdesc','gen3_2f85'],
   'Enabled':['HandoverInterceptionController'],'Timestep':.001,'Plugins':[],
   'ClearControllerModulePath':True,'ControllerModulePaths':[str(module)],
   'GUIServer':{'Enable':True,'TCP':{'Host':'127.0.0.1','Ports':[42420,43430]}},'LogDirectory':str(out)}
(out/'global.yaml').write_text(yaml.safe_dump(g))
manifest={'commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
 'diff':subprocess.check_output(['git','diff'],cwd=root,text=True), 'scenario':a.scenario,
 'controller_sha256':hashlib.sha256((a.build/'src/libHandoverInterceptionController.so').read_bytes()).hexdigest(),
 'purpose':'diagnostic, not H1/outcome inference'}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
env=dict(os.environ); env['LD_LIBRARY_PATH']=str(a.build.resolve()/'src')+':'+env.get('LD_LIBRARY_PATH','')
start=time.monotonic()
with (out/'run.log').open('w') as f:
    proc=subprocess.Popen(['/home/harry/mc_rtc_ws/install/bin/mc_rtc_ticker','-f',str(out/'global.yaml')],stdout=f,stderr=subprocess.STDOUT,env=env)
    time.sleep(1)
    if proc.poll() is None:
        maps=P(f'/proc/{proc.pid}/maps').read_text()
        (out/'loaded_libraries.txt').write_text('\n'.join(x for x in maps.splitlines() if 'HandoverInterception' in x))
        if str(a.build.resolve()/'src/libHandoverInterceptionController.so') not in maps:
            proc.terminate(); proc.wait(); raise RuntimeError('wrong controller library loaded')
    # This installation faults in ticker teardown after run_for returns.
    # Use the historical runner's explicit terminal/timeout termination policy;
    # preserve exit status and distinguish it from spontaneous crashes.
    stop_reason = 'spontaneous_exit'
    while proc.poll() is None:
        elapsed = time.monotonic()-start
        content = (out/'run.log').read_text(errors='replace')
        if '[Completed] full plan-once handover completed' in content or 'Starting state HandoverInterceptionController_Failure' in content:
            stop_reason = 'wrapper_terminal'; break
        if elapsed > a.seconds+60:
            stop_reason = 'wrapper_timeout'; break
        time.sleep(.25)
    if proc.poll() is None:
        proc.terminate()
        try: rc=proc.wait(timeout=5)
        except subprocess.TimeoutExpired: proc.kill(); rc=proc.wait()
    else: rc=proc.returncode
    manifest['stop_reason']=stop_reason

manifest.update(exit_code=rc,wall_seconds=time.monotonic()-start)
(out/'manifest.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(manifest | {'diff':'see manifest'},indent=2))
