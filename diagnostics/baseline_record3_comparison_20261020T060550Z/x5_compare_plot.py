import json,os
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
D=Path(Path('/tmp/x5_comparison_task').read_text().strip());r=json.loads((D/'report.json').read_text());f=json.loads((D/'features.json').read_text());q=np.load(D/'paths.npz')
plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
fig,axs=plt.subplots(2,2,figsize=(12,8.5),layout='constrained')
colors={'baseline':'#db6c28','record3_raw':'#2166ac'};labels={'baseline':'Standalone Baseline','record3_raw':'record3 (recorded joints)'}
for name in colors:
 color=colors[name];x=np.array(f[name]['tip_xyz_m'])
 axs[0,0].plot(x[:,0],x[:,2],color=color,label=labels[name],lw=2)
 axs[0,0].scatter(x[0,0],x[0,2],color=color,s=45)
 axs[0,0].annotate('start',(x[0,0],x[0,2]),xytext=(4,-15),textcoords='offset points',color=color)
 axs[1,0].plot(np.linspace(0,100,801),np.degrees(q[name][:,4]),color=color,lw=2,label=labels[name])
 axs[1,1].plot(np.linspace(0,100,201),1000*np.array(f[name]['distal_clearance_estimate_m']),color=color,lw=2,label=labels[name])
 v=r['results'][name]['per_joint_excess_deg'];shift=-.18 if name=='baseline' else .18
 axs[0,1].bar(np.arange(1,8)+shift,np.maximum(v,0),.36,color=color,label=labels[name])
end=np.array(f['baseline']['tip_xyz_m'])[-1];axs[0,0].scatter(end[0],end[2],c='black',marker='*',s=110,zorder=4)
axs[0,0].axvline(end[0],c='grey',ls=':',lw=1);axs[0,0].annotate('same goal',(end[0],end[2]),xytext=(-65,8),textcoords='offset points')
axs[0,0].set(xlabel='Base-frame x (m)',ylabel='Height z (m)',title='Stick-tip path: different starting poses');axs[0,0].set_aspect('equal',adjustable='datalim');axs[0,0].legend(loc='lower right')
axs[0,1].set(xlabel='Right-arm joint',ylabel='Extra angular travel (degrees)',title='Total joint travel minus net start-to-goal change',xticks=np.arange(1,8))
axs[1,0].set(xlabel='Outward joint-arc progress (%) — not time',ylabel='Joint 5 angle (degrees)',title='Baseline J5 first moves away from its goal');axs[1,0].axhline(np.degrees(q['baseline'][-1,4]),c='grey',ls=':',lw=1)
axs[1,1].set(xlabel='Outward joint-arc progress (%) — not time',ylabel='Distal FCL padding-band estimate (mm)',title='Same saved camera scene; lower means closer')
for ax in axs.flat:ax.grid(alpha=.2)
fig.suptitle('record3 vs latest standalone Baseline — offline geometry, not physical execution',fontsize=14)
fig.savefig(D/'comparison.png',dpi=150);plt.close(fig)
