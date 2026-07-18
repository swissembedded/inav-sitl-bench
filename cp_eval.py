import csv, math, statistics as st
def up(r_deg, p_deg):
    r, p = math.radians(r_deg), math.radians(p_deg)
    return (-math.sin(p), math.sin(r)*math.cos(p), math.cos(r)*math.cos(p))
for name in ('cp_magonly.csv','cp_both.csv'):
    rows=list(csv.DictReader(open(name)))
    seg=[r for r in rows if r['phase']=='circle']
    late=seg[len(seg)//3:]
    yerr=[abs((float(r['fc_yaw'])-float(r['js_yaw'])%360+540)%360-180) for r in late]
    rerr=[float(r['fc_roll'])-float(r['js_roll']) for r in late]
    perr=[float(r['fc_pitch'])-float(r['js_pitch']) for r in late]
    tilt=[]
    for r in late:
        uf=up(float(r['fc_roll']),float(r['fc_pitch']))
        uj=up(float(r['js_roll']),float(r['js_pitch']))
        d=max(-1.0,min(1.0,sum(a*b for a,b in zip(uf,uj))))
        tilt.append(math.degrees(math.acos(d)))
    bank=st.median(abs(float(r['js_roll'])) for r in late)
    print(f"{name}: bank {bank:4.0f} | yaw med {st.median(yerr):6.1f} max {max(yerr):6.1f} | "
          f"roll-bias {st.median(rerr):+5.1f} pitch-bias {st.median(perr):+5.1f} | tilt med {st.median(tilt):4.1f} max {max(tilt):4.1f}")
