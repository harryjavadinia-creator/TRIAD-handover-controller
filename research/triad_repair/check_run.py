#!/usr/bin/env python3
"""Safety/contract checks for TRIAD-lite diagnostic logs, including failed runs.
Success is not required. These checks do not assert a valid-at-use certificate.
"""
import argparse,json,re,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def check(text):
    lines=text.splitlines(); errors=[]
    commits=[i for i,l in enumerate(lines) if '[V2TerminalCommit]' in l and 'committed=true' in l]
    if len(commits)>1: errors.append('multiple commitments')
    if 'geometry_mismatch' in text: errors.append('geometry contract violation')
    if 'giverTruthModel=independent_scripted' not in text: errors.append('independent giver not established')
    if '[TriadRepairConfig]' in text and 'matched=true' in text:
        for l in lines:
            if '[TriadRepairConfig]' in l and 'matched=true' in l:
                if 'authority=diagnostic admissionAuthority=false' not in l: errors.append('authority setting mismatch')
    commit=commits[0] if commits else len(lines)
    for i,l in enumerate(lines):
        if i<commit and 'closureAuthorized=true' in l: errors.append('premature closure authorization')
        if i>commit and ('[V2PlanningJobSubmit]' in l or '[V2ProvisionalAdopt]' in l): errors.append('postcommit planning/adoption')
        if '[V2PostCommitReselectionRefused]' in l: errors.append('postcommit reselection attempt')
        if '[CommitInterceptionPlan]' in l: errors.append('V1 commitment path')
    # Candidate logs explicitly contain OBJECT-relative transforms.
    poses={}
    for l in lines:
        if '[TriadLiteCandidate]' not in l or 'family=receiving' not in l: continue
        d=dict(re.findall(r'(\w+)=(\[[^\]]*\]|[^ ]+)',l))
        if 'O_T_G_q' not in d: continue
        g=int(d['id']); p=[float(v) for v in d['O_T_G_p'][1:-1].split(',')]; q=[float(v) for v in d['O_T_G_q'][1:-1].split(',')]
        if g in poses:
            oldp,oldq=poses[g]
            dp=sum((x-y)**2 for x,y in zip(p,oldp))**.5
            dq=min(sum((x-y)**2 for x,y in zip(q,oldq)),sum((x+y)**2 for x,y in zip(q,oldq)))**.5
            if dp>3e-6 or dq>3e-6: errors.append('physical ID drift '+str(g))
        poses[g]=(p,q)
    completed='[Completed] full plan-once handover completed' in text
    if completed and len(commits)!=1: errors.append('completion without one commitment')
    return {'passed':not errors,'errors':sorted(set(errors)),'commits':len(commits),'completed':completed,'observed_physical_ids':len(poses)}
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('log',type=Path);p.add_argument('--mutations',action='store_true');a=p.parse_args()
    text=a.log.read_text(); result=check(text)
    replay=subprocess.run([sys.executable,'-B',str(ROOT/'tools/check_giver_truth_independence.py'),'--replay',str(a.log)],capture_output=True,text=True)
    result['giver_replay']=replay.stdout.strip(); result['passed']=result['passed'] and replay.returncode==0
    if a.mutations:
        for name,line in [('geometry','[TriadRepairContract] geometry_mismatch'),('closure','closureAuthorized=true'),('v1','[CommitInterceptionPlan]')]:
            assert not check(line+'\n'+text)['passed'],name
        assert not check(text+'\n[V2TerminalCommit] committed=true\n[V2TerminalCommit] committed=true')['passed']
        result['mutation_tests']='passed geometry, premature closure, V1 path, multiple commitments'
    print(json.dumps(result,indent=2));sys.exit(0 if result['passed'] else 1)
