#!/usr/bin/env python3
"""
figuras_seccion5.py — Figuras de la sección 5 (Modelos y simulaciones) del informe del 25 %.

  fig_latencia_modos.png   latencia SensorTimestamp -> matriz vs período de cuadro (bench_camara.py)
  fig_error_pix_ang.png    error de la aproximación lineal Δθ = e·IFOV frente a atan (pt/geometria.py)
  fig_barrido_rls.png      barrido de λ y orden del RLS (barrido_rls_ruido.py)

Uso (desde la carpeta rpi5_esqueleto):
  python3 herramientas/figuras_seccion5.py <carpeta_de_salida>
"""
import math, sys, os
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT=sys.argv[1] if len(sys.argv)>1 else "resultados/figuras_seccion5"; os.makedirs(OUT,exist_ok=True)
C=["#2a78d6","#eb6834","#1baf7a","#eda100"]; INK,INK2,GRID,SURF="#0b0b0b","#52514e","#e4e3dd","#ffffff"
plt.rcParams.update({"font.size":9,"axes.edgecolor":INK2,"axes.labelcolor":INK,"xtick.color":INK2,"ytick.color":INK2,"text.color":INK,"axes.titlesize":9,"axes.titleweight":"bold","font.family":"DejaVu Sans"})
def limpiar(ax):
    for s in ("top","right"): ax.spines[s].set_visible(False)
    ax.grid(color=GRID,linewidth=0.6); ax.set_axisbelow(True)

# ---------- Fig 1: latencia vs periodo de cuadro del modo ----------
fr=[]
for d in ("bench_rgb","bench_yuv","bench_exp2ms"):
    r=pd.read_csv(f"resultados/{d}/resumen.csv"); r["corrida"]=d; fr.append(r)
r=pd.concat(fr); r["T"]=1000/r.fps_max_modo
a,b=np.polyfit(r["T"],r.lat_med_ms,1); R2=np.corrcoef(r["T"],r.lat_med_ms)[0,1]**2
fig,ax=plt.subplots(figsize=(6.2,3.4),dpi=220)
xx=np.linspace(0,50,10)
ax.plot(xx,xx,"--",color=INK2,lw=1,label="L = T (referencia)")
ax.plot(xx,a*xx+b,"-",color=C[1],lw=2,label=f"ajuste: L = {a:.2f}·T − {abs(b):.1f} ms  (R² = {R2:.3f})")
ax.errorbar(r["T"],r.lat_med_ms,yerr=[np.zeros(len(r)),r.lat_p95_ms-r.lat_med_ms],fmt="o",ms=6,color=C[0],mec=SURF,mew=1.2,ecolor=C[0],elinewidth=1,capsize=0,label="mediana (barra hasta p95), 60 config.")
nombres={0:"640×480 (recorte)\n200 fps",1:"1640×1232 (binning)\n81 fps",2:"1920×1080 (recorte)\n47,6 fps".replace(",","."),3:"3280×2464 (completo)\n21 fps"}
for m,(dx,dy,ha) in {0:(1.3,-1,"left"),1:(1.3,-4.5,"left"),2:(1.3,-5,"left"),3:(-1.5,-6,"right")}.items():
    s=r[r.modo.isin([m,m+4])]; ax.text(s["T"].iloc[0]+dx,s.lat_med_ms.median()+dy,nombres[m],fontsize=7.5,color=INK,ha=ha,va="top")
ax.set_xlabel("Período de cuadro del modo del sensor, T [ms]"); ax.set_ylabel("Latencia SensorTimestamp → matriz [ms]")
ax.set_xlim(0,50); ax.set_ylim(0,65); limpiar(ax); ax.legend(frameon=False,fontsize=7.5,loc="upper left")
fig.tight_layout(); fig.savefig(f"{OUT}/fig_latencia_modos.png",bbox_inches="tight"); plt.close(fig)
print("fit",a,b,R2)

# ---------- Fig 2: error aproximacion lineal pixel->angulo ----------
sys.path.insert(0,".")
from pt.parametros import Parametros; from pt.geometria import Geometria
g=Geometria.desde_parametros(Parametros())
u=np.linspace(g.cx,639,400)
ex=np.array([g.pix_a_ang(x,g.cy)[0] for x in u]); li=np.array([g.pix_a_ang_lineal(x,g.cy)[0] for x in u])
err=li-ex; du=u-g.cx
# diagonal (esquina): error en pan y tilt
k=np.linspace(0,1,400); ud=g.cx+k*(639-g.cx); wd=g.cy-k*(g.cy-0)
exd=np.array([g.pix_a_ang(a_,b_) for a_,b_ in zip(ud,wd)]); lid=np.array([g.pix_a_ang_lineal(a_,b_) for a_,b_ in zip(ud,wd)])
errd=np.hypot(*(lid-exd).T)
paso=0.1125; i1=np.argmax(err>paso); print("err>paso en du=",du[i1], "ang", ex[i1])
fig,ax=plt.subplots(figsize=(6.2,3.2),dpi=220)
ax.plot(du,err,color=C[0],lw=2,label="eje horizontal (tilt = 0)")
ax.plot(k*(639-g.cx),errd,color=C[1],lw=2,label="diagonal hacia la esquina (norma pan-tilt)")
ax.axhline(paso,ls="--",color=INK2,lw=1); ax.text(3,paso*1.25,"1 micropaso = 0.1125°",fontsize=7.5,color=INK2)
ax.plot([du[i1]],[paso],"o",ms=7,color=C[0],mec=SURF,mew=1.5)
ax.annotate(f"{du[i1]:.0f} px del centro ({ex[i1]:.1f}°)",(du[i1],paso),xytext=(125,0.025),fontsize=7.5,arrowprops=dict(arrowstyle="-",color=INK2,lw=0.8))
ax.set_yscale("log"); ax.set_ylim(1e-3,10)
ax.set_xlabel("Distancia horizontal del objetivo al centro de la imagen [px]"); ax.set_ylabel("Error de Δθ = e·IFOV frente a atan [°]")
limpiar(ax); ax.legend(frameon=False,fontsize=7.5,loc="lower right")
fig.tight_layout(); fig.savefig(f"{OUT}/fig_error_pix_ang.png",bbox_inches="tight"); plt.close(fig)
for x in (400,480,560,639):
    e_,l_=g.pix_a_ang(x,g.cy)[0],g.pix_a_ang_lineal(x,g.cy)[0]; print(x,round(e_,2),round(l_,2),round(l_-e_,2))
print("esquina", g.pix_a_ang(639,0), g.pix_a_ang_lineal(639,0))

# ---------- Fig 3: barrido RLS ----------
d=pd.read_csv("resultados/barrido_rls/datos_barrido_rls.csv"); n=pd.read_csv("resultados/barrido_rls/datos_barrido_ruido.csv")
fig,axs=plt.subplots(1,3,figsize=(8.0,3.2),dpi=220,sharey=True)
ax=axs[0]
for j,(h,lab) in enumerate([(50.0,"h = 50 ms (informe 5 %)"),(26.94,"h = 26.9 ms (τ_eq medido)")]):
    s=d[(d.fps==60)&(d.h_ms==h)&(d.orden==2)].groupby("lam")[["rms_pred","rms_sin"]].mean()
    ax.plot(s.index,s.rms_pred,"-o",ms=4,color=C[j],lw=2,mec=SURF,mew=1,label=lab)
    ax.axhline(s.rms_sin.iloc[0],ls="--",color=C[j],lw=1)
ax.set_title("(a) 60 fps, orden 2\nsin ruido añadido"); ax.legend(frameon=False,fontsize=6.5,loc="upper left")
ax.text(0.95,0.12,"discontinua: sin predictor",fontsize=6.5,color=INK2,ha="right")
for ax,orden in ((axs[1],2),(axs[2],3)):
    for j,sg in enumerate([0.0,0.25,0.5,1.0]):
        s=n[(n.orden==orden)&(n.sigma_px==sg)].groupby("lam")[["rms_pred","rms_sin"]].mean()
        ax.plot(s.index,s.rms_pred,"-o",ms=4,color=C[j],lw=2,mec=SURF,mew=1,label=f"σ = {sg} px")
    ax.axhspan(2.86,3.20,color=GRID,alpha=0.9,lw=0); ax.text(0.5,3.6,"sin predictor (2.9–3.2 px)",fontsize=6.5,color=INK2)
    ax.axvline(0.80,color=INK2,lw=0.8,ls=":")
    ax.set_title(f"({'b' if orden==2 else 'c'}) 80 fps, h = 24.9 ms\norden {orden}")
axs[1].legend(frameon=False,fontsize=6.5,loc="lower right",title="ruido de detección",title_fontsize=6.5)
for ax in axs:
    ax.set_yscale("log"); ax.set_ylim(0.07,20); ax.set_xlabel("Factor de olvido λ"); limpiar(ax)
    ax.set_xticks([0.5,0.6,0.7,0.8,0.9])
axs[0].set_ylabel("Error RMS de posición en t+h [px]")
fig.tight_layout(); fig.savefig(f"{OUT}/fig_barrido_rls.png",bbox_inches="tight"); plt.close(fig)

# CSV de respaldo

