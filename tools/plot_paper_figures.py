from pathlib import Path
import sys,json,math,os
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).resolve().parent/'qa/matplotlib-cache'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,FancyBboxPatch,FancyArrowPatch,Arc
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
from matplotlib import font_manager
for ff in ['times.ttf','timesbd.ttf','timesi.ttf','timesbi.ttf']:
 local_font=Path(__file__).resolve().parent/'fonts'/ff
 if not local_font.exists():local_font=Path(r'C:\Windows\Fonts')/ff
 if local_font.exists():font_manager.fontManager.addfont(str(local_font))
font_manager.findfont('Times New Roman',fallback_to_default=False)
P=Path(__file__).resolve().parents[1]/'results/paper_data';F=P/'figures';F.mkdir(parents=True,exist_ok=True);A=P/'analysis'
plt.rcParams.update({'font.family':'Times New Roman','mathtext.fontset':'custom','mathtext.rm':'Times New Roman','mathtext.it':'Times New Roman:italic','mathtext.bf':'Times New Roman:bold','mathtext.cal':'Times New Roman:italic','mathtext.sf':'Times New Roman','mathtext.tt':'Times New Roman','mathtext.fallback':None,'font.size':9,'axes.labelsize':8.5,'axes.titlesize':9,'legend.fontsize':8,'xtick.labelsize':7.5,'ytick.labelsize':7.5,'pdf.fonttype':42,'ps.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,'savefig.bbox':'tight','axes.linewidth':.65})
from palette import LINES, FILLS, INK, BORDER, GRAY
C=LINES[:4];red=LINES[4];grey=BORDER;ink=INK
def load(id):return json.loads((A/f'{id}_trace.json').read_text())
def arr(x):return np.asarray(x,float)
def legs(z):return z['legs'] if isinstance(z['legs'],list) else [z['legs']]
def save(fig,name):
 fig.canvas.draw()
 groups={}
 for ax in fig.axes:
  group=ax.get_subplotspec().rowspan.start if hasattr(ax,'get_subplotspec') else 0
  groups.setdefault(group,[]).append(ax)
 for group,aa in groups.items():
  y=min(ax.get_position().y0 for ax in aa)
  for ax in aa:
   for tx in ax.texts:
    if getattr(tx,'_panel_title',False):
     tx.xy=(.5,(y-ax.get_position().y0)/ax.get_position().height);tx.xycoords='axes fraction'
 # Pack the actual rendered extents, including axes labels and subcaptions.
 # Equal-aspect maps otherwise leave much larger gaps than GridSpec suggests.
 fig.canvas.draw(); renderer=fig.canvas.get_renderer()
 structured=[a for a in fig.axes if hasattr(a,'get_subplotspec')]
 if len(structured)>1:
  columns={}; rows={}
  for a in structured:
   spec=a.get_subplotspec();columns.setdefault(spec.colspan.start,[]).append(a);rows.setdefault(spec.rowspan.start,[]).append(a)
  def extent(group):
   bb=[a.get_tightbbox(renderer) for a in group]
   return min(b.x0 for b in bb),max(b.x1 for b in bb),min(b.y0 for b in bb),max(b.y1 for b in bb)
  original=extent(structured); previous=None; gap=8*fig.dpi/72
  for key in sorted(columns):
   group=columns[key];bb=extent(group)
   if previous is not None:
    shift=(previous+gap-bb[0])/fig.bbox.width
    for a in group:
     p=a.get_position();a.set_position([p.x0+shift,p.y0,p.width,p.height])
    fig.canvas.draw();renderer=fig.canvas.get_renderer();bb=extent(group)
   previous=bb[1]
  packed=extent(structured);shift=((original[0]+original[1])-(packed[0]+packed[1]))/2/fig.bbox.width
  for a in structured:
   p=a.get_position();a.set_position([p.x0+shift,p.y0,p.width,p.height])
  fig.canvas.draw();renderer=fig.canvas.get_renderer();previous=None;gap=7*fig.dpi/72
  for key in sorted(rows):
   group=rows[key];bb=extent(group)
   if previous is not None:
    shift=(previous-gap-bb[3])/fig.bbox.height
    for a in group:
     p=a.get_position();a.set_position([p.x0,p.y0+shift,p.width,p.height])
    fig.canvas.draw();renderer=fig.canvas.get_renderer();bb=extent(group)
   previous=bb[2]
 # Rasterize dense geometry only; keep selected paths, axes and all text as vectors.
 for ax in fig.axes:
  for line in ax.lines:
   if len(line.get_xdata())>300 and line.get_zorder()<3:line.set_rasterized(True)
  for coll in ax.collections:coll.set_rasterized(True)
 fig.savefig(F/f'{name}.pdf',dpi=300,pad_inches=.025);fig.savefig(F/f'{name}.png',dpi=180,pad_inches=.025);plt.close(fig)
def corners(q,t,margin=0):
 q=arr(q);xy=np.array([[t['front']+margin,t['width']/2+margin],[t['front']+margin,-t['width']/2-margin],[-t['rear']-margin,-t['width']/2-margin],[-t['rear']-margin,t['width']/2+margin]])
 R=np.array([[np.cos(q[2]),-np.sin(q[2])],[np.sin(q[2]),np.cos(q[2])]])
 return xy@R.T+q[:2]
def body(ax,q,t,c,margin=0,fill=True):
 ax.add_patch(Polygon(corners(q,t,margin),facecolor=matplotlib.colors.to_rgba(c,.16) if fill else 'none',edgecolor=c,lw=.9,ls=(0,(3,2)) if margin else '-',zorder=5))
class View:
 def __init__(self,z,angle=None):
  sc=z['scene'];self.origin=arr(sc['goal'])[:2];d=arr(sc['start'])[:2]-self.origin;self.a=math.atan2(d[1],d[0]) if angle is None else angle;self.R=np.array([[np.cos(self.a),np.sin(self.a)],[-np.sin(self.a),np.cos(self.a)]])
 def xy(self,p):return (arr(p)[...,:2]-self.origin)@self.R.T
 def pose(self,q):
  q=arr(q).copy();q[...,:2]=self.xy(q);q[...,2]-=self.a;return q
 def bounds(self,clouds,poses=(),truck=None,pad=5,aspect=1.9):
  xy=[self.xy(q).reshape(-1,2) for q in clouds if arr(q).size]
  if truck:
   xy.extend([self.xy(corners(q,truck,.5)) for q in poses])
  p=np.vstack(xy);lo=p.min(0)-pad;hi=p.max(0)+pad;d=hi-lo;c=(lo+hi)/2
  if d[0]/d[1]<aspect:d[0]=d[1]*aspect
  else:d[1]=d[0]/aspect
  return [c[0]-d[0]/2,c[0]+d[0]/2,c[1]-d[1]/2,c[1]+d[1]/2]
def frame(ax,z,v,lim,labels=True):
 sc=z['scene'];b=v.xy(sc['boundary']);ax.plot(b[:,0],b[:,1],'.',c=GRAY,ms=1.05,zorder=1)
 pk=arr(sc['parkedTruck'])
 if pk.size:
  for q in pk.reshape(-1,3):body(ax,v.pose(q),sc['truck'],GRAY)
 for name,c in [('start',C[0]),('goal',C[3])]:
  q=v.pose(sc[name]);body(ax,q,sc['truck'],c);body(ax,q,sc['truck'],c,.5,False)
  ax.annotate('',xy=q[:2]+5*np.array([np.cos(q[2]),np.sin(q[2])]),xytext=q[:2],arrowprops={'arrowstyle':'->','color':c,'lw':1},zorder=6)
  if labels and lim[0]<q[0]<lim[1] and lim[2]<q[1]<lim[3]:
   foot=corners(q,sc['truck'],.5);lo=foot.min(0);hi=foot.max(0);cx=foot[:,0].mean();dy=(lim[3]-lim[2])*.02;dx=(lim[1]-lim[0])*.018
   candidates=[[cx,lo[1]-dy],[lo[0]-2*dx,lo[1]-dy],[hi[0]+2*dx,lo[1]-dy],[cx,lo[1]-4*dy]]
   occupied=[corners(v.pose(pp),sc['truck']) for pp in pk.reshape(-1,3)] if pk.size else []
   labelpt=candidates[0]
   for pt in candidates:
    if not (lim[0]+dx<pt[0]<lim[1]-dx and lim[2]+2*dy<pt[1]<lim[3]):continue
    overlap=any(pt[0]+dx>o[:,0].min() and pt[0]-dx<o[:,0].max() and pt[1]>o[:,1].min() and pt[1]-2*dy<o[:,1].max() for o in occupied)
    if not overlap:labelpt=pt;break
   ax.annotate('S' if name=='start' else 'G',labelpt,ha='center',va='top',c=c,weight='bold',fontsize=8,zorder=8,bbox={'fc':'white','ec':'none','pad':.2})
 ax.set_aspect('equal');ax.set_xlim(*lim[:2]);ax.set_ylim(*lim[2:]);ax.set_xlabel('x (m)',labelpad=1);ax.set_ylabel('y (m)',labelpad=1);ax.grid(alpha=.18,lw=.45);ax.xaxis.set_major_locator(MaxNLocator(5));ax.yaxis.set_major_locator(MaxNLocator(4))
def curve(ax,v,q,**kw):
 q=v.xy(q)
 if kw.get('lw',1)<1:kw.setdefault('rasterized',True)
 return ax.plot(q[:,0],q[:,1],**kw)
def sol(ax,z,v):
 for l in legs(z):curve(ax,v,l['q'],c=C[0] if l['gear']>0 else C[1],lw=1.65,zorder=4)
 for l1,l2 in zip(legs(z)[:-1],legs(z)[1:]):
  if l1['gear']!=l2['gear']:
   q=v.xy(arr(l1['q'])[-1]);ax.plot(*q,'s',ms=4,c=red,zorder=7)
def title(ax,s,offset=33):
 tx=ax.annotate(s,xy=(.5,0),xycoords='axes fraction',xytext=(0,-offset),textcoords='offset points',ha='center',va='top',fontsize=9);tx._panel_title=True
# Common architecture is drawn by the dedicated layout script.
from draw_architecture import draw as draw_architecture
draw_architecture()
# Actual Branch I computation, shown in a common rigidly rotated display frame.
z=load('B058');v=View(z);dp=z['dp'];poses=arr(dp['poses']);qc=arr(dp['coarse']['q']);qf=arr(dp['fine']['q']);fp=arr(dp['finePoses'])
lim=v.bounds([poses,qc,qf]+[l['q'] for l in legs(z)],poses=[z['scene']['start'],z['scene']['goal']],truck=z['scene']['truck'],pad=4,aspect=2.2)
fig,axs=plt.subplots(2,2,figsize=(7.16,3.85));fig.subplots_adjust(left=.065,right=.995,bottom=.18,top=.96,hspace=.83,wspace=.19)
for ax,tt in zip(axs.flat,['(a) Coarse states and headings','(b) Curves tested by lazy DP','(c) Refined sampling corridor','(d) Retained layered path']):frame(ax,z,v,lim);title(ax,tt)
ax=axs[0,0];p=v.pose(poses);ax.scatter(p[:,0],p[:,1],s=5,c=C[1],alpha=.5);ax.quiver(p[:,0],p[:,1],3*np.cos(p[:,2]),3*np.sin(p[:,2]),angles='xy',scale_units='xy',scale=1,width=.002,color=C[1],alpha=.4)
for ec in dp['edgeCurves']:
 state=ec['state'];curve(axs[0,1],v,ec['q'],c=C[0] if state==2 else (red if state==-1 else grey),alpha=.3 if state==2 else .18,lw=.55)
curve(axs[0,1],v,qc,c=C[0],lw=1.7,zorder=4)
ax=axs[1,0];nvec=np.array([-np.sin(dp['axis']),np.cos(dp['axis'])]);band=np.vstack([qc[:,:2]+3*nvec,(qc[:,:2]-3*nvec)[::-1]]);ax.add_patch(Polygon(v.xy(band),fc=grey,ec='none',alpha=.4));p=v.xy(fp);ax.plot(p[:,0],p[:,1],'.',c=C[1],ms=2,alpha=.45);curve(ax,v,qc,ls='--',c=GRAY,lw=.8);curve(ax,v,qf,c=C[0],lw=1.6);sol(axs[1,1],z,v)
fig.legend(handles=[Line2D([0],[0],color=C[0],label='validated'),Line2D([0],[0],color=red,label='rejected'),Line2D([0],[0],color=grey,label='pending')],loc='upper center',ncol=3,frameon=False,bbox_to_anchor=(.55,1.04),handlelength=1.3,columnspacing=1.3,fontsize=7.7)
save(fig,'dp_steps')
# Two branch rows: generated fans, safe fans, and complete selected path.
fig,axs=plt.subplots(2,3,figsize=(7.16,4.65),gridspec_kw={'width_ratios':[1,1,.7]});fig.subplots_adjust(left=.065,right=.995,bottom=.14,top=.99,hspace=.52,wspace=.26)
for row,id in enumerate(['B018','B055']):
 z=load(id);v=View(z,angle=0);f=z['fan'];fans=f['fans'];cloud=[t['q'] for t in fans];lim=v.bounds(cloud,poses=[z['scene']['goal']],truck=z['scene']['truck'],pad=6,aspect=1.45)
 limpath=v.bounds([l['q'] for l in legs(z)],poses=[z['scene']['start'],z['scene']['goal']],truck=z['scene']['truck'],pad=7,aspect=.67)
 for col in range(3):
  ax=axs[row,col];frame(ax,z,v,lim if col<2 else limpath)
  if col<2:
   for fan in fans:
    if col==1 and not fan['safe']:continue
    curve(ax,v,fan['q'],c=C[0] if fan['safe'] else red,lw=.58,alpha=.5 if fan['safe'] else .24)
   anchors=np.unique(arr([t['anchor'] for t in fans]),axis=0);aa=v.xy(anchors);ax.plot(aa[:,0],aa[:,1],'D',c=ink,ms=3,zorder=6)
   ax.annotate(r'$A_o$' if row==0 else 'anchors',aa[-1],xytext=(12,8),textcoords='offset points',fontsize=8,color=ink,zorder=9,bbox={'fc':'white','ec':'none','pad':.3},arrowprops={'arrowstyle':'-','lw':.5})
  else:
   sol(ax,z,v);cc=v.xy(f['cusp']);ax.annotate(r'$q_c$',cc,xytext=(4,4),textcoords='offset points',color=red,fontsize=9)
  txt=[f'All {len(fans)} fans',f'{sum(t["safe"] for t in fans)} safe fans','Selected path'][col]
  title(ax,f'({chr(97+row*3+col)}) '+txt)
  if col==0:ax.text(.04,.95,'Branch II' if row==0 else 'Branch III',transform=ax.transAxes,ha='left',va='top',fontsize=9,fontweight='bold',bbox={'fc':'white','ec':'none','pad':1})
save(fig,'fan_steps')
# Prefix detail panels and complete final maneuvers.
fig,axs=plt.subplots(2,3,figsize=(7.16,4.05));fig.subplots_adjust(left=.065,right=.995,bottom=.17,top=.98,hspace=.64,wspace=.3)
for row,id in enumerate(['B027','B030']):
 z=load(id);v=View(z);nn=z['nodes'];nn=nn if isinstance(nn,list) else [nn];ls=legs(z);count=int(z['prefixLegCount'])
 local=v.bounds([n['q'] for n in nn],poses=[z['scene']['start'],arr(ls[count-1]['q'])[-1]],truck=z['scene']['truck'],pad=2.5,aspect=1.45)
 whole=v.bounds([l['q'] for l in ls],poses=[z['scene']['start'],z['scene']['goal']],truck=z['scene']['truck'],pad=4,aspect=1.45)
 for col in range(3):
  ax=axs[row,col];frame(ax,z,v,local if col<2 else whole)
  if col<2:
   for n in nn:
    if col==0 and n['depth']!=1:continue
    curve(ax,v,n['q'],c=C[0] if n['safe'] else red,lw=.85,alpha=.45 if n['safe'] else .25);p=v.xy(arr(n['q'])[-1]);ax.plot(*p,'.',c=C[0] if n['safe'] else red,ms=3)
   if col==1:
    for l in ls[:count]:curve(ax,v,l['q'],c=C[0] if l['gear']>0 else C[1],lw=2.1,zorder=6)
    child=v.pose(arr(ls[count-1]['q'])[-1]);body(ax,child,z['scene']['truck'],C[2]);ax.plot(*child[:2],'D',ms=4,c=C[2],zorder=7);ax.annotate(r"$q'_s$",child[:2],xytext=(8,9),textcoords='offset points',fontsize=9,color=C[2],arrowprops={'arrowstyle':'->','lw':.65})
  else:sol(ax,z,v)
  title(ax,f'({chr(97+row*3+col)}) '+['First-level trials','Selected restart','Complete trajectory'][col])
 axs[row,0].text(.04,.96,f'Case #{int(id[1:])}',transform=axs[row,0].transAxes,ha='left',va='top',fontsize=9,fontweight='bold',bbox={'fc':'white','ec':'none','pad':1})
save(fig,'recursion')
# Five representations on the same task and execution-time decomposition.
rows=json.loads((A/'metrics100.json').read_text());methods=['BRS','HA','BIAGT','GHA','CC-A*'];labels=['BRS','Hybrid A*','BIAGT','GHA-search','CC-A*'];colors=LINES
z=json.loads((P/'inputs/B003_comparison.json').read_text());v=View(z);lim=v.bounds([m['q'] for m in z['methods']],poses=[z['scene']['start'],z['scene']['goal']],truck=z['scene']['truck'],pad=3,aspect=1.55)
fig,axs=plt.subplots(2,3,figsize=(7.16,4.65));fig.subplots_adjust(left=.065,right=.995,bottom=.15,top=.97,hspace=.62,wspace=.31)
for k,ax in enumerate(axs.flat):
 if k==5:continue
 frame(ax,z,v,lim);r=z['methods'][k];curve(ax,v,r['q'],c=colors[k],lw=1.6)
 events=r['events'];events=events if isinstance(events,list) else ([events] if events else [])
 for e in events:
  if 1e-8<e['distance']<r['length']-1e-8:
   n=int(np.argmin(abs(arr(r['s'])-e['distance'])));p=v.xy(arr(r['q'])[n]);ax.plot(*p,'s' if e['shift'] else 'o',c=red if e['shift'] else C[3],ms=3.5,zorder=7,markeredgecolor='white',markeredgewidth=.3)
 score=next(x for x in rows if x['id']=='B003' and x['method']==methods[k]);title(ax,f'({chr(97+k)}) {labels[k]}',offset=29)
 ax.text(.04,.95,f'$T_f$={score["totalTime"]:.1f} s\n$N_s$={score["extraStops"]}',transform=ax.transAxes,ha='left',va='top',fontsize=8,bbox={'fc':'white','ec':'none','alpha':.85,'pad':1})
ax=axs.flat[5];rr=[next(x for x in rows if x['id']=='B003' and x['method']==m) for m in methods];y=np.arange(5)
ax.barh(y,[r['movingTime'] for r in rr],color=FILLS[0],edgecolor=INK,linewidth=.45,height=.56,label='Moving')
ax.barh(y,[r['dwellTime'] for r in rr],left=[r['movingTime'] for r in rr],color=FILLS[3],edgecolor=INK,linewidth=.45,height=.56,label='Stationary')
ax.set_yticks(y,labels,fontsize=7.5);ax.invert_yaxis();ax.set_xlabel('Predicted execution time (s)');ax.grid(axis='x',alpha=.2);ax.legend(loc='lower right',fontsize=7,frameon=False)
title(ax,'(f) Time decomposition',offset=29);save(fig,'example_comparison')

# Connector geometry: labels follow the actual piece structure.
z=load('B018');p=z['fan']['connector'];q0=arr(p['start']);segments=arr(p['segments']);q=[q0.copy()];ss=[0];ks=[0];bounds=[0];ga,gw=np.polynomial.legendre.leggauss(16)
for L,k,sigma in segments:
 start=q[-1].copy();base=ss[-1];ts=np.linspace(0,L,max(3,int(np.ceil(L/.1))+1))
 for x in ts[1:]:
  u=(ga+1)*x/2;th=start[2]+k*u+.5*sigma*u*u;dz=x/2*np.array([np.dot(gw,np.cos(th)),np.dot(gw,np.sin(th))]);q.append(np.r_[start[:2]+dz,start[2]+k*x+.5*sigma*x*x]);ss.append(base+x);ks.append(k+sigma*x)
 bounds.append(len(q)-1)
q=arr(q);ss=arr(ss);ks=arr(ks);q[:,:2]-=q0[:2]
straight=np.flatnonzero((abs(segments[:,1])<1e-14)&(abs(segments[:,2])<1e-14));si=int(straight[np.argmax(segments[straight,0])])
# Rotate the display frame only; preserve the exact connector and curvature.
display_angle=-np.pi/2;rotation=np.array([[0,-1],[1,0]])
q[:,:2]=q[:,:2]@rotation.T;q[:,2]-=display_angle
fig,axs=plt.subplots(1,2,figsize=(7.16,2.3),gridspec_kw={'width_ratios':[1,1.25]});ax=axs[0]
for j,(a,b) in enumerate(zip(bounds[:-1],bounds[1:])):ax.plot(q[a:b+1,0],q[a:b+1,1],color=C[0] if j<si else (C[3] if j==si else C[1]),lw=1.8)
ax.annotate(r'$q_a$',q[0,:2],xytext=(-6,-10),textcoords='offset points');ax.annotate(r'$q_b$',q[-1,:2],xytext=(3,-10),textcoords='offset points')
mid=(q[bounds[si],:2]+q[bounds[si+1],:2])/2;beta=float(p['middleHeading'])-display_angle;scale=max(np.ptp(q[:,0]),np.ptp(q[:,1]));radius=.13*scale
ax.plot([mid[0],mid[0]+radius*1.4],[mid[1],mid[1]],'k:',lw=.7);ax.annotate('+x',(mid[0]+radius*1.4,mid[1]),xytext=(1,2),textcoords='offset points',fontsize=7)
ang=math.degrees(math.atan2(math.sin(beta),math.cos(beta)));ax.add_patch(Arc(mid,2*radius,2*radius,theta1=min(0,ang),theta2=max(0,ang),lw=.8))
at=mid+radius*1.2*np.array([np.cos(beta/2),np.sin(beta/2)]);ax.text(*at,r'$\beta$',fontsize=10)
ax.annotate('',xy=mid+radius*1.4*np.array([np.cos(beta),np.sin(beta)]),xytext=mid,arrowprops={'arrowstyle':'->','color':C[3]})
ax.annotate(r'$d$',mid,xytext=(0,6),textcoords='offset points',color=C[3],fontsize=10)
circles=np.flatnonzero((abs(segments[:,1])>1e-12)&(abs(segments[:,2])<1e-14))
if len(circles):
 ci=int(circles[0]);j=(bounds[ci]+bounds[ci+1])//2;R=1/segments[ci,1];cen=q[j,:2]+R*np.array([-np.sin(q[j,2]),np.cos(q[j,2])]);ax.plot([cen[0],q[j,0]],[cen[1],q[j,1]],'k--',lw=.65);ax.annotate(r'$R$',(cen+q[j,:2])/2,xytext=(3,2),textcoords='offset points',fontsize=9)
ax.set_aspect('equal');ax.margins(.15);ax.set_xlabel('x relative to $q_a$ (m)');ax.set_ylabel('y relative to $q_a$ (m)');title(ax,'(a) Connector geometry');ax.grid(alpha=.2)
ax=axs[1];ax.plot(ss,ks,c=C[0],lw=1.5);ax.text(.98,.94,r'$|\kappa|\leq 1/R$',transform=ax.transAxes,ha='right',va='top',fontsize=8);ax.axhline(0,c=GRAY,lw=.6);ax.set_xlabel('Arc length s (m)');ax.set_ylabel(r'$\kappa$ (m$^{-1}$)');title(ax,'(b) Curvature profile')
for sign in [-1,1]:
 j=int(np.argmin(ks) if sign<0 else np.argmax(ks))
 if abs(ks[j])>1e-8:ax.annotate(r'$-k_p$' if sign<0 else r'$+k_p$',(ss[j],ks[j]),xytext=(3,-12 if sign<0 else 4),textcoords='offset points',fontsize=8)
ax.annotate(r'$L_r$',(ss[bounds[1]]/2,ks[bounds[1]]/2),xytext=(12,10),textcoords='offset points',arrowprops={'arrowstyle':'->','lw':.6},fontsize=9)
ax.text((ss[bounds[si]]+ss[bounds[si+1]])/2,.005,r'$d$',color=C[3]);ax.margins(y=.25);ax.grid(alpha=.2)
fig.tight_layout(rect=[0,.18,1,1]);save(fig,'connector')

# Every successful observation contributes to the CDF; failures remain above its terminal plateau.
fig,axs=plt.subplots(1,2,figsize=(7.16,2.5))
for m,c,l in zip(methods,colors,labels):
 rr=[x for x in rows if x['method']==m and x['success']];x=np.sort([r['qualityGapPercent'] for r in rr]);axs[0].plot(x,np.arange(1,len(x)+1)/100,c=c,label=l,lw=1.3)
 x=np.sort([r['totalPlanningSeconds'] for r in rr]);axs[1].semilogx(x,np.arange(1,len(x)+1)/100,c=c,label=l,lw=1.3)
axs[0].set(xlabel='Relative cost gap (%)',ylabel='Fraction of 100 tasks');axs[1].set(xlabel='Planning wall-clock time (s)',ylabel='Fraction of 100 tasks');axs[0].legend(loc='lower right',fontsize=7.6)
for ax in axs:ax.grid(alpha=.2);ax.set_ylim(0,1.015)
title(axs[0],'(a) Relative trajectory cost');title(axs[1],'(b) Successful planning time');fig.tight_layout(rect=[0,.15,1,1]);save(fig,'results_cdf')

# Full refinement population, including rejected fine candidates.
ab=json.loads((A/'refinement_records.json').read_text());valid=[x for x in ab if isinstance(x['fineJ'],(int,float)) and np.isfinite(x['fineJ'])]
fig,ax=plt.subplots(figsize=(3.5,2.35));fig.subplots_adjust(left=.18,right=.99,bottom=.22,top=.95)
x=np.arange(1,len(ab)+1);delta=np.array([r['fineJ']-r['coarseJ'] if isinstance(r['fineJ'],(int,float)) and np.isfinite(r['fineJ']) else np.nan for r in ab]);retained=np.array([d if r['selectedFine'] else 0 for d,r in zip(delta,ab)])
ax.vlines(x,0,delta,color=FILLS[4],lw=.8);ax.plot(x,delta,'o',ms=3,c=red,label='Refined candidate');ax.plot(x,retained,'s',ms=2.7,c=C[0],label='Retained result')
missing=~np.isfinite(delta)
if np.any(missing):ax.plot(x[missing],np.zeros(sum(missing)),'x',ms=4,c=ink,label='No fine path')
ax.axhline(0,lw=.7,c=ink);ax.set_xlabel('Refinement attempt');ax.set_ylabel(r'Cost change $\Delta J$ (s)');ax.grid(alpha=.18)
for target in ['B058','B079','B086']:
 for j,r in enumerate(ab):
  if r['id']==target and abs(delta[j])>.1:ax.annotate(f'#{int(target[1:])}',(j+1,delta[j]),xytext=(1,5),textcoords='offset points',fontsize=7)
ax.legend(fontsize=7,frameon=False,loc='upper left');ax.margins(y=.18);save(fig,'ablation')
print('Generated all numerical figures from the completed experiment.')
