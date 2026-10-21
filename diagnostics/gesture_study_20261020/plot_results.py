import sys,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from drum_planner.experiment.runtime import Oracle
from drum_planner.experiment.config import no_can,METHODS
from drum_planner.experiment.gesture import fk
from drum_planner import style
no_can();root=Path(__file__).resolve().parent;old=root.parent/'baseline_record3_comparison_20261020T060550Z'
folder=root/'refine/main-seed19';result=json.loads((folder/'result.json').read_text());evaluation=json.loads((folder/'evaluation.json').read_text())
features=json.loads((old/'features.json').read_text());metrics=json.loads((root/'reference_geometry.json').read_text())
paths={'Standalone baseline':features['baseline'],'Recorded (different start)':features['record3_raw']}
data={'Standalone baseline':dict(gesture=metrics['baseline'],duration_s=5.778400325), 'Recorded (different start)':dict(gesture=metrics['record3_raw'],duration_s=7.095750341)}
colors={'Standalone baseline':'#888888','Recorded (different start)':'#111111','Joint progress':'#d07615','Broad sweep':'#12805c','Single arc':'#1d64bf','Clearance blend':'#8e4aaa'}
labels=dict(progress='Joint progress',sweep='Broad sweep',single_arc='Single arc',clearance='Clearance blend')
o=Oracle(folder/'acquisition_request.json',root)
try:
 for mode in METHODS:
  row=result['selected'][mode];c=json.loads((folder/row['file']).read_text());q=style.sample_path(np.array([p['positions_rad'] for p in c['outward']]))
  paths[labels[mode]]=fk(o,q);data[labels[mode]]=evaluation['methods'][mode]
finally:o.close()
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axes=plt.subplots(2,2,figsize=(14,10),layout='constrained')
for ax,name in zip(axes[0],('tip','wrist')):
 for label,f in paths.items():
  x=np.array(f[name+'_xyz_m']);ax.plot(x[:,0],x[:,2],label=label,c=colors[label],lw=1.8,ls='--' if 'Recorded' in label else '-')
  ax.scatter(*x[0,[0,2]],c=colors[label],s=18)
 ax.set_xlabel('Robot-base X (m)');ax.set_ylabel('Robot-base Z (m)');ax.set_title(name.capitalize()+' route (X–Z projection)');ax.grid(alpha=.2)
axes[0,0].legend(fontsize=8)
labels=list(paths);x=np.arange(len(labels));short=['Baseline','Record3','Progress','Sweep','Single arc','Clearance']
axes[1,0].bar(x,[data[k]['gesture']['joint_excess_deg'] for k in labels],color=[colors[k] for k in labels]);axes[1,0].set_xticks(x,short,rotation=20);axes[1,0].set_ylabel('Extra motor travel (degrees)');axes[1,0].set_title('Backtracking beyond net endpoint change (lower better)')
axes[1,1].bar(x,[data[k]['duration_s'] for k in labels],color=[colors[k] for k in labels]);axes[1,1].set_xticks(x,short,rotation=20);axes[1,1].set_ylabel('Predicted outward playback (seconds)');axes[1,1].set_title('Timing trade-off — no speed-based pruning')
fig.suptitle('Frozen-scene study — main task, seed 19\nBaseline and generated paths share endpoints; record3 starts elsewhere. No physical execution.',fontsize=14)
fig.savefig(root/'main_comparison.png',dpi=160);plt.close(fig)
# Paired held-out scatter: plot every test, not just favorable examples.
rows=[json.loads(p.read_text()) for p in (root/'heldout').glob('*/evaluation.json')]
fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained')
for mode in METHODS:
 points=[r['methods'][mode] for r in rows];label=labels[mode] if isinstance(labels,dict) else dict(progress='Joint progress',sweep='Broad sweep',single_arc='Single arc',clearance='Clearance blend')[mode]
 axes[0].scatter([r['gesture']['joint_excess_deg'] for r in points],[r['similarity']['tip_chord_residual_rms_cm'] for r in points],s=30,alpha=.65,color=colors[label],label=label)
 axes[1].scatter([r['gesture']['wrist']['detour_ratio'] for r in points],[r['proximity'] for r in points],s=30,alpha=.65,color=colors[label],label=label)
axes[0].set_xlabel('Extra motor travel (degrees)');axes[0].set_ylabel('Record3 tip shape residual RMS (cm)');axes[0].set_title('Held-out simplicity vs recording shape (both lower better)');axes[0].legend(fontsize=8)
axes[1].set_xlabel('Wrist path / endpoint displacement');axes[1].set_ylabel('Soft proximity penalty');axes[1].set_title('Held-out route economy vs proximity (both lower better)')
for ax in axes:ax.grid(alpha=.2)
fig.savefig(root/'heldout_tradeoffs.png',dpi=160);plt.close(fig)
print('Plots saved',flush=True)
