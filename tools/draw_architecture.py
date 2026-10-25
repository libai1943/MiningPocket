from pathlib import Path
import os,sys
P=Path(__file__).resolve().parents[1]/'results/paper_data';(P/'figures').mkdir(parents=True,exist_ok=True)
os.environ.setdefault('MPLCONFIGDIR',str(Path(__file__).resolve().parent/'qa/matplotlib-cache'))
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch,FancyArrowPatch,Polygon,Rectangle
from matplotlib import font_manager
for name in ['times.ttf','timesbd.ttf','timesi.ttf','timesbi.ttf']:
    f=P/'fonts'/name
    if not f.exists():f=Path(r'C:\Windows\Fonts')/name
    if f.exists():font_manager.fontManager.addfont(str(f))
font_manager.findfont('Times New Roman',fallback_to_default=False)
plt.rcParams.update({'font.family':'Times New Roman','font.size':8.5,'pdf.fonttype':42,'ps.fonttype':42,'mathtext.fontset':'custom','mathtext.rm':'Times New Roman','mathtext.it':'Times New Roman:italic','mathtext.bf':'Times New Roman:bold'})
from palette import INK, GRAY, BORDER, CYAN, BLUE, LAVENDER, WHEAT, ROSE
ink=INK;muted=GRAY;green='#00a2e8';blue='#3f48cc';purple='#a349a4';amber='#b97a57'

def draw():
    fig,ax=plt.subplots(figsize=(7.16,4.35));fig.subplots_adjust(0,0,1,1);ax.set(xlim=(0,180),ylim=(-1,111));ax.axis('off')
    def arrow(points,color=ink,width=.85):
        for a,b in zip(points[:-2],points[1:-1]):ax.plot([a[0],b[0]],[a[1],b[1]],color=color,lw=width,zorder=4)
        ax.add_patch(FancyArrowPatch(points[-2],points[-1],arrowstyle='-|>',mutation_scale=7,lw=width,color=color,zorder=4,shrinkA=0,shrinkB=0))
    def box(x,y,w,h,title,body='',color=ink,fill='#f8fafb',fs=9):
        ax.add_patch(FancyBboxPatch((x-w/2,y-h/2),w,h,boxstyle='round,pad=0,rounding_size=1.3',fc=fill,ec=color,lw=.7,zorder=3))
        if body:
            ax.text(x,y+h*.22,title,color=ink,ha='center',va='center',weight='bold',fontsize=fs,zorder=4)
            ax.text(x,y-h*.17,body,color=ink,ha='center',va='center',fontsize=8.05,linespacing=1.2,zorder=4)
        else:ax.text(x,y,title,color=color,ha='center',va='center',weight='bold',fontsize=fs,zorder=4)
    def note(x,y,t,color=muted,fs=7.7):
        ax.text(x,y,t,ha='center',va='center',fontsize=fs,color=color,zorder=5,bbox={'fc':'white','ec':'none','pad':.5})
    box(91,105,128,8,'Current start  |  Goal and static map  |  Retained prefix',fs=9.1)
    arrow([(91,101),(91,98),(42,98),(42,91)])
    arrow([(91,101),(91,91)])
    arrow([(91,98),(140,98),(140,91)])
    cards=[(42,green,'I','Reverse-only',['Direct connection and layers','Corridor refinement','Keep the lower-cost route'],'Zero cusps'),(91,blue,'II','Goal-side',['Goal anchors and direct entry','Reverse-entry fan family','Forward connector to cusp'],'At most one cusp'),(140,purple,'III','Start-side',['Forward departure family','Sampled cusp poses','Reverse connector to goal'],'At most one cusp')]
    for ix,(x,c,roman,title,steps,foot) in enumerate(cards):
        ax.add_patch(FancyBboxPatch((x-22,57),44,34,boxstyle='round,pad=0,rounding_size=1.35',fc='white',ec=BORDER,lw=.75,zorder=3))
        ax.add_patch(FancyBboxPatch((x-22,83),44,8,boxstyle='round,pad=0,rounding_size=1.35',fc=[CYAN,BLUE,LAVENDER][ix],ec='none',zorder=3))
        ax.plot([x-20.6,x+20.6],[90.7,90.7],lw=1.6,c=c,zorder=4)
        ax.text(x,87,roman+'   '+title,ha='center',va='center',fontsize=10,weight='bold',color=ink,zorder=4)
        for j,t in enumerate(steps):
            y=79.5-7*j;ax.text(x,y,t,ha='center',va='center',fontsize=8.4,color=ink,zorder=4)
            if j<2:arrow([(x,y-2.1),(x,y-4.7)],c,.65)
        ax.text(x,59.8,foot,ha='center',va='center',fontsize=7.7,fontstyle='italic',color=ink,zorder=4)
        arrow([(x,57),(x,49)],c,.8)
    box(91,43,112,12,'Common trajectory evaluation and selection','Footprint and endpoint checks  |  Speed profile  |  Minimum J',color=ink,fill='#f2f2f2',fs=9.2)
    arrow([(91,37),(91,35)])
    ax.add_patch(Polygon([[91,35],[103,27],[91,19],[79,27]],fc='white',ec=ink,lw=.8,zorder=3));ax.text(91,27,'Feasible\nshortlist?',ha='center',va='center',fontsize=8.3,zorder=4)
    box(144,27,51,18,'Complete trajectory','Verify prefix + local route\nRetain the best sibling by J',color=green,fill=CYAN,fs=9)
    arrow([(103,27),(118.5,27)],green);note(110.5,30.5,'yes',green)
    box(36,27,48,16,'Broaden II / III','Use the next sampling level',color=amber,fill=WHEAT,fs=9)
    arrow([(79,27),(60,27)],amber);note(69.5,30.5,'no',amber)
    # The local-level return only supplies the two one-cusp constructions.
    arrow([(12,27),(7.5,27),(7.5,94),(103,94),(103,91)],amber,.8)
    arrow([(103,94),(152,94),(152,91)],amber,.8)
    note(57,94,'II / III sampling',ink,7.6)
    # Once a local invocation is exhausted, resume the breadth-first prefix queue.
    arrow([(36,19),(36,15)],amber)
    note(61,17,'local levels exhausted',amber,7.5)
    box(67,7,116,14,'Breadth-first prefix continuation','Expand queued parent: gear × steering × length\nValidated child → new start; retain the complete prefix',color=amber,fill=WHEAT,fs=9)
    arrow([(9,7),(2,7),(2,105),(27,105)],amber,1.0)
    box(153,7,45,13,'Terminate if exhausted','Queue or search limits',color=muted,fill='#f2f2f2',fs=8.4)
    arrow([(125,7),(130.5,7)],muted,.75)
    # Align the three branch panels and all routing rails precisely.
    fig.savefig(P/'figures/architecture.pdf',bbox_inches='tight',pad_inches=.025,dpi=300)
    fig.savefig(P/'figures/architecture.png',bbox_inches='tight',pad_inches=.025,dpi=180)
    plt.close(fig)
if __name__=='__main__':draw()