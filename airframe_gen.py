# Copyright (C) 2026 Daniel Haensse
# GPL-3.0-or-later (see repo LICENSE)
"""Parametric JSBSim airframe generator for the bench hangar. Emits the
same idealized-thruster FDM structure as the hand-written aerobat3d /
turbotimber / kingfisher / dragonfly, from a compact parameter dict -
specs researched per airframe, aero derived from geometry class.

    python airframe_gen.py            # (re)generate all
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))

# name: dict(desc, span, area, m_empty, m_batt, thrust_lbf, thr_xz,
#            torque_lbf, cl0, clmax, a_max_deg, clmin, cd0, k_ind,
#            cm0, cm_a, cm_ele, roll_ail, yaw_rud, pw_ele, pw_rud, pw_ail,
#            dihedral, iyy_scale)
AIRFRAMES = {
    # Multiplex EasyGlider 4: 1.8 m Elapor motor glider, 41.6 dm2,
    # ~1.1 kg RR, AR 7.8 - floaty, low power, big rudder authority.
    "easyglider": dict(
        desc="Multiplex EasyGlider 4, 1.8 m Elapor motor glider",
        span=1.80, area=0.416, m_empty=0.88, m_batt=0.22,
        thrust_lbf=2.2, thr_xz=(0.30, 0.0), torque_lbf=0.05,
        cl0=0.40, clmax=1.30, a_max_deg=12, clmin=-0.55,
        cd0=0.035, k_ind=0.052,
        cm0=0.035, cm_a=-0.70, cm_ele=-0.55, roll_ail=0.14,
        yaw_rud=0.16, pw_ele=-4.0, pw_rud=3.5, pw_ail=0.0,
        dihedral=-0.12, iyy_scale=1.0),
    # Multiplex EasyStar 3: 1.37 m pusher-pod trainer, 28 dm2, 0.70 kg,
    # ailerons since v3; the pod sits above the wing -> mild nose-down
    # throttle couple, prop stream misses the tail.
    "easystar": dict(
        desc="Multiplex EasyStar 3, 1.37 m pusher-pod trainer",
        span=1.37, area=0.28, m_empty=0.56, m_batt=0.14,
        thrust_lbf=1.0, thr_xz=(-0.05, 0.06), torque_lbf=0.03,
        cl0=0.40, clmax=1.25, a_max_deg=13, clmin=-0.45,
        cd0=0.045, k_ind=0.058,
        cm0=0.035, cm_a=-0.65, cm_ele=-0.45, roll_ail=0.10,
        yaw_rud=0.13, pw_ele=-0.8, pw_rud=0.8, pw_ail=0.0,
        dihedral=-0.11, iyy_scale=1.0),
    # Multiplex Xeno Uni: 1.245 m flying wing, 32 dm2, ~0.65 kg electric.
    # ELEVONS ONLY - no rudder, no fin authority beyond the winglets:
    # yaw_rud is ZERO, knife-edge is physically impossible here (the
    # airframe case for capability-based mode gating). Short pitch arm.
    "xeno": dict(
        desc="Multiplex Xeno Uni, 1.245 m flying wing (elevons, no rudder)",
        span=1.245, area=0.32, m_empty=0.51, m_batt=0.14,
        thrust_lbf=1.3, thr_xz=(0.05, 0.03), torque_lbf=0.03,
        cl0=0.15, clmax=1.05, a_max_deg=14, clmin=-0.70,
        cd0=0.030, k_ind=0.085,
        cm0=0.010, cm_a=-0.25, cm_ele=-0.28, roll_ail=0.20,
        yaw_rud=0.0, pw_ele=-1.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.03, iyy_scale=0.6),
    # HobbyZone AeroScout S 2 1.1m: 1.095 m EPO pusher trainer, ~0.81 kg,
    # 2306/2250Kv on 5x4.5 (3S) - pod above the wing, gentle.
    "aeroscout": dict(
        desc="HobbyZone AeroScout S 2, 1.1 m pusher trainer",
        span=1.095, area=0.20, m_empty=0.64, m_batt=0.17,
        thrust_lbf=1.7, thr_xz=(-0.05, 0.07), torque_lbf=0.04,
        cl0=0.38, clmax=1.25, a_max_deg=13, clmin=-0.45,
        cd0=0.050, k_ind=0.060,
        cm0=0.035, cm_a=-0.60, cm_ele=-0.45, roll_ail=0.11,
        yaw_rud=0.12, pw_ele=-0.8, pw_rud=0.8, pw_ail=0.0,
        dihedral=-0.10, iyy_scale=1.0),
    # E-flite A-10 Thunderbolt II Twin 64mm EDF: 1.149 m, 21.9 dm2,
    # 2.32 kg (6S) - 106 g/dm2 loading, twin EDF on the fuselage: NO wash
    # over any surface, thrust near the CG line, flies on airspeed alone.
    "a10": dict(
        desc="E-flite A-10 twin 64mm EDF, 1.15 m - EDF, zero propwash",
        span=1.149, area=0.219, m_empty=1.92, m_batt=0.40,
        thrust_lbf=5.4, thr_xz=(-0.05, 0.02), torque_lbf=0.0,
        cl0=0.15, clmax=1.15, a_max_deg=14, clmin=-0.75,
        cd0=0.040, k_ind=0.070,
        cm0=0.010, cm_a=-0.50, cm_ele=-0.60, roll_ail=0.16,
        yaw_rud=0.10, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.02, iyy_scale=1.2, flap_cl=0.40),
    # ParkZone Icon A5: 1.33 m amphibian, ~1.23 kg, 480/960Kv 9x8 pusher
    # on the pylon - thrust well above the CG (nose-down couple), hull
    # side area, the dragonfly's grown-up sibling.
    "icona5": dict(
        desc="ParkZone Icon A5, 1.33 m amphibian pylon pusher",
        span=1.330, area=0.26, m_empty=1.01, m_batt=0.22,
        thrust_lbf=2.1, thr_xz=(-0.05, 0.10), torque_lbf=0.05,
        cl0=0.35, clmax=1.25, a_max_deg=13, clmin=-0.50,
        cd0=0.048, k_ind=0.058,
        cm0=0.030, cm_a=-0.55, cm_ele=-0.50, roll_ail=0.13,
        yaw_rud=0.13, pw_ele=-0.8, pw_rud=0.8, pw_ail=0.0,
        dihedral=-0.09, iyy_scale=1.0),
    # ParkZone Bf 109G: 1.105 m warbird, ~1.0 kg on a narrow wing
    # (~55 g/dm2) - fast, torque-heavy, sharper post-stall break than the
    # trainers (warbird tip-stall temperament).
    "bf109": dict(
        desc="ParkZone Messerschmitt Bf 109G, 1.1 m warbird",
        span=1.105, area=0.185, m_empty=0.84, m_batt=0.16,
        thrust_lbf=2.4, thr_xz=(0.30, 0.0), torque_lbf=0.08,
        cl0=0.25, clmax=1.20, a_max_deg=12, clmin=-0.60,
        cd0=0.038, k_ind=0.062,
        cm0=0.020, cm_a=-0.50, cm_ele=-0.55, roll_ail=0.15,
        yaw_rud=0.12, pw_ele=-5.0, pw_rud=4.0, pw_ail=1.5,
        dihedral=-0.05, iyy_scale=1.0),
    # Freewing Lippisch P.15 Diana 64mm EDF: 750 mm tailless delta with a
    # FIXED fin - EDF (no wash), light, T/W ~1.4. ELEVONS ONLY (Daniel):
    # no rudder servo, yaw_rud zero like the xeno.
    "lippisch": dict(
        desc="Freewing Lippisch P.15 Diana 64mm EDF, 750 mm tailless delta",
        span=0.75, area=0.13, m_empty=0.26, m_batt=0.16,
        thrust_lbf=1.3, thr_xz=(-0.05, 0.0), torque_lbf=0.0,
        cl0=0.10, clmax=1.00, a_max_deg=16, clmin=-0.70,
        cd0=0.032, k_ind=0.090,
        cm0=0.005, cm_a=-0.30, cm_ele=-0.35, roll_ail=0.18,
        yaw_rud=0.0, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.03, iyy_scale=0.7),
    # Freewing MiG-15 64mm EDF: 700 mm swept-wing jet, 470 g - EDF, no
    # wash, warbird-jet loading (~47 g/dm2). The RC model has aileron and
    # elevator ONLY (Daniel) - the fin is fixed, no rudder servo.
    "mig15": dict(
        desc="Freewing MiG-15 64mm EDF, 700 mm swept jet",
        span=0.70, area=0.10, m_empty=0.34, m_batt=0.13,
        thrust_lbf=1.3, thr_xz=(-0.05, 0.0), torque_lbf=0.0,
        cl0=0.12, clmax=1.05, a_max_deg=14, clmin=-0.75,
        cd0=0.035, k_ind=0.080,
        cm0=0.008, cm_a=-0.45, cm_ele=-0.50, roll_ail=0.15,
        yaw_rud=0.0, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.02, iyy_scale=1.0),
    # Dynam PT-17 Stearman 1300 mm: BIPLANE - two wings worth of area and
    # drag on 2.15 kg (4S), gentle and slow, big prop torque.
    "pt17": dict(
        desc="Dynam PT-17 Stearman, 1.3 m biplane",
        span=1.30, area=0.55, m_empty=1.80, m_batt=0.35,
        thrust_lbf=2.9, thr_xz=(0.35, 0.0), torque_lbf=0.10,
        cl0=0.35, clmax=1.30, a_max_deg=13, clmin=-0.45,
        cd0=0.075, k_ind=0.075,
        cm0=0.030, cm_a=-0.55, cm_ele=-0.50, roll_ail=0.11,
        yaw_rud=0.14, pw_ele=-6.0, pw_rud=5.0, pw_ail=1.0,
        dihedral=-0.08, iyy_scale=1.0),
    # Sonicmodell Binary 1200 mm: EPO twin-motor FPV/survey platform,
    # ~1.8 kg loaded - twins modeled as one centered thruster (no
    # differential yaw in the plant yet), zero wash over the tail.
    "binary": dict(
        desc="Sonicmodell Binary 1200 mm twin-motor FPV platform",
        span=1.20, area=0.26, m_empty=1.45, m_batt=0.35,
        thrust_lbf=3.3, thr_xz=(0.10, 0.0), torque_lbf=0.0,
        cl0=0.30, clmax=1.25, a_max_deg=13, clmin=-0.50,
        cd0=0.045, k_ind=0.062,
        cm0=0.025, cm_a=-0.55, cm_ele=-0.50, roll_ail=0.13,
        yaw_rud=0.12, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.06, iyy_scale=1.0, flap_cl=0.45),
    # Sonicmodell AR Wing Pro: 1000 mm EPP FPV flying wing - elevons
    # only, no rudder (like the xeno, faster and heavier).
    "arwing": dict(
        desc="Sonicmodell AR Wing Pro, 1.0 m FPV flying wing (no rudder)",
        span=1.00, area=0.26, m_empty=0.60, m_batt=0.25,
        thrust_lbf=1.8, thr_xz=(-0.10, 0.02), torque_lbf=0.04,
        cl0=0.12, clmax=1.05, a_max_deg=14, clmin=-0.70,
        cd0=0.028, k_ind=0.075,
        cm0=0.008, cm_a=-0.25, cm_ele=-0.28, roll_ail=0.20,
        yaw_rud=0.0, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.03, iyy_scale=0.6),
    # ZOHD Delta Strike: 600 mm EPP delta with a 50 mm EDF - tiny, hot,
    # elevons plus a small fin rudder, no wash.
    "deltastrike": dict(
        desc="ZOHD Delta Strike, 600 mm EPP EDF delta",
        span=0.60, area=0.12, m_empty=0.45, m_batt=0.20,
        thrust_lbf=1.1, thr_xz=(-0.08, 0.0), torque_lbf=0.0,
        cl0=0.10, clmax=1.00, a_max_deg=16, clmin=-0.70,
        cd0=0.034, k_ind=0.095,
        cm0=0.005, cm_a=-0.28, cm_ele=-0.30, roll_ail=0.18,
        yaw_rud=0.05, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.03, iyy_scale=0.7),
    # Durafly D.H.100 Vampire V3 (RCAF Silver): 1100 mm twin-boom jet,
    # 1050 g, 70 mm 5-blade EDF - no wash, broad straight wing
    # (~46 g/dm2). Aileron + elevator ONLY (Daniel): the twin fins are
    # fixed, no rudder servo.
    "vampire": dict(
        desc="Durafly DH.100 Vampire V3, 1.1 m twin-boom 70mm EDF",
        span=1.10, area=0.23, m_empty=0.83, m_batt=0.22,
        thrust_lbf=3.0, thr_xz=(-0.05, 0.0), torque_lbf=0.0,
        cl0=0.18, clmax=1.10, a_max_deg=14, clmin=-0.70,
        cd0=0.036, k_ind=0.070,
        cm0=0.012, cm_a=-0.48, cm_ele=-0.55, roll_ail=0.15,
        yaw_rud=0.0, pw_ele=0.0, pw_rud=0.0, pw_ail=0.0,
        dihedral=-0.04, iyy_scale=1.0),
}

TPL = """<?xml version="1.0"?>
<!--
  {name}: {desc}.
  GENERATED by airframe_gen.py (GPL-3.0-or-later, see repo LICENSE) -
  edit the parameter dict there, not this file. Same idealized-thruster
  structure as aerobat3d; coefficients are handbook estimates derived
  from researched specs (span {span} m, S {area} m2, AUW ~{auw:.2f} kg,
  static thrust ~{thrust_kg:.2f} kg => T/W ~{tw:.2f}).
-->
<fdm_config name="{name}" version="2.0" release="ALPHA">
  <fileheader>
    <author>Daniel Haensse</author>
    <description>{desc}</description>
  </fileheader>
  <metrics>
    <wingarea unit="M2"> {area} </wingarea>
    <wingspan unit="M"> {span} </wingspan>
    <chord unit="M"> {chord:.3f} </chord>
    <htailarea unit="M2"> {htail:.3f} </htailarea>
    <htailarm unit="M"> {arm:.2f} </htailarm>
    <vtailarea unit="M2"> {vtail:.3f} </vtailarea>
    <vtailarm unit="M"> {arm:.2f} </vtailarm>
    <location name="AERORP" unit="M"> <x> 0.0 </x> <y> 0 </y> <z> 0 </z> </location>
    <location name="EYEPOINT" unit="M"> <x> 0 </x> <y> 0 </y> <z> 0.1 </z> </location>
    <location name="VRP" unit="M"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
  </metrics>
  <mass_balance>
    <ixx unit="KG*M2"> {ixx:.4f} </ixx>
    <iyy unit="KG*M2"> {iyy:.4f} </iyy>
    <izz unit="KG*M2"> {izz:.4f} </izz>
    <emptywt unit="KG"> {m_empty} </emptywt>
    <location name="CG" unit="M"> <x> 0 </x> <y> 0 </y> <z> 0 </z> </location>
    <pointmass name="battery">
      <weight unit="KG"> {m_batt} </weight>
      <location unit="M"> <x> 0.05 </x> <y> 0 </y> <z> 0 </z> </location>
    </pointmass>
  </mass_balance>
  <ground_reactions>
    <contact type="BOGEY" name="MAIN_L">
      <location unit="M"> <x> 0.05 </x> <y> -0.18 </y> <z> -0.15 </z> </location>
      <static_friction> 0.7 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 800 </spring_coeff> <damping_coeff unit="N/M/SEC"> 60 </damping_coeff>
    </contact>
    <contact type="BOGEY" name="MAIN_R">
      <location unit="M"> <x> 0.05 </x> <y> 0.18 </y> <z> -0.15 </z> </location>
      <static_friction> 0.7 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 800 </spring_coeff> <damping_coeff unit="N/M/SEC"> 60 </damping_coeff>
    </contact>
    <contact type="BOGEY" name="TAIL">
      <location unit="M"> <x> -0.50 </x> <y> 0 </y> <z> -0.05 </z> </location>
      <static_friction> 0.7 </static_friction> <dynamic_friction> 0.5 </dynamic_friction>
      <spring_coeff unit="N/M"> 400 </spring_coeff> <damping_coeff unit="N/M/SEC"> 30 </damping_coeff>
    </contact>
  </ground_reactions>
  <external_reactions>
    <force name="motor" frame="BODY" unit="LBS">
      <function>
        <product>
          <property> fcs/throttle-cmd-norm </property>
          <value> {thrust_lbf} </value>
        </product>
      </function>
      <location unit="M"> <x> {thr_x} </x> <y> 0 </y> <z> {thr_z} </z> </location>
      <direction> <x> 1 </x> <y> 0 </y> <z> 0 </z> </direction>
    </force>
    <force name="prop-torque-r" frame="BODY" unit="LBS">
      <function>
        <product>
          <property> fcs/throttle-cmd-norm </property>
          <value> {torque_lbf} </value>
        </product>
      </function>
      <location unit="M"> <x> {thr_x} </x> <y> 0.4 </y> <z> {thr_z} </z> </location>
      <direction> <x> 0 </x> <y> 0 </y> <z> -1 </z> </direction>
    </force>
    <force name="prop-torque-l" frame="BODY" unit="LBS">
      <function>
        <product>
          <property> fcs/throttle-cmd-norm </property>
          <value> {torque_lbf} </value>
        </product>
      </function>
      <location unit="M"> <x> {thr_x} </x> <y> -0.4 </y> <z> {thr_z} </z> </location>
      <direction> <x> 0 </x> <y> 0 </y> <z> 1 </z> </direction>
    </force>
  </external_reactions>
  <flight_control name="FCS">
    <property value="0.0"> fcs/throttle-cmd-norm </property>
    <property value="0.0"> fcs/flap-cmd-norm </property>
    <channel name="Roll">
      <aerosurface_scale name="aileron">
        <input>fcs/aileron-cmd-norm</input>
        <range> <min>-1</min> <max>1</max> </range>
        <output>fcs/aileron-pos-norm</output>
      </aerosurface_scale>
    </channel>
    <channel name="Pitch">
      <aerosurface_scale name="elevator">
        <input>fcs/elevator-cmd-norm</input>
        <range> <min>-1</min> <max>1</max> </range>
        <output>fcs/elevator-pos-norm</output>
      </aerosurface_scale>
    </channel>
    <channel name="Yaw">
      <aerosurface_scale name="rudder">
        <input>fcs/rudder-cmd-norm</input>
        <range> <min>-1</min> <max>1</max> </range>
        <output>fcs/rudder-pos-norm</output>
      </aerosurface_scale>
    </channel>
  </flight_control>
  <aerodynamics>
    <axis name="LIFT">
      <function name="aero/force/Lift_alpha">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <table>
            <independentVar lookup="row">aero/alpha-rad</independentVar>
            <tableData>
              -3.142   0.00
              -1.571   0.00
              -0.524  {clmin_brk:.2f}
              -0.349  {clmin:.2f}
              -0.175  {clmin_half:.2f}
               0.000  {cl0:.2f}
               {a_half:.3f}  {cl_half:.2f}
               {a_max:.3f}  {clmax:.2f}
               0.436  {cl_post:.2f}
               0.524  {cl_post2:.2f}
               1.571   0.00
               3.142   0.00
            </tableData>
          </table>
        </product>
      </function>
      <function name="aero/force/Lift_elevator">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>fcs/elevator-pos-norm</property> <value> 0.12 </value>
        </product>
      </function>{flap_lift}
    </axis>
    <axis name="DRAG">
      <function name="aero/force/Drag_basic">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <value> {cd0} </value>
        </product>
      </function>
      <function name="aero/force/Drag_induced">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>aero/cl-squared</property> <value> {k_ind} </value>
        </product>
      </function>{flap_drag}
      <function name="aero/force/Drag_alpha_poststall">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <table>
            <independentVar lookup="row">aero/alpha-rad</independentVar>
            <tableData>
              -3.142   0.00
              -1.571   1.30
              -1.047   1.00
              -0.524   0.30
              -0.262   0.00
               0.262   0.00
               0.524   0.30
               1.047   1.00
               1.571   1.30
               3.142   0.00
            </tableData>
          </table>
        </product>
      </function>
    </axis>
    <axis name="SIDE">
      <function name="aero/force/Side_beta">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>aero/beta-rad</property> <value> -0.40 </value>
        </product>
      </function>
    </axis>
    <axis name="ROLL">
      <function name="aero/moment/Roll_damp">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>aero/bi2vel</property>
          <property>velocities/p-aero-rad_sec</property> <value> -0.40 </value>
        </product>
      </function>
      <function name="aero/moment/Roll_aileron">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>fcs/aileron-pos-norm</property>
          <value> {roll_ail} </value>
        </product>
      </function>
      <function name="aero/moment/Roll_dihedral">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>aero/beta-rad</property>
          <value> {dihedral} </value>
        </product>
      </function>
    </axis>
    <axis name="PITCH">
      <function name="aero/moment/Pitch_zero">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/cbarw-ft</property>
          <value> {cm0} </value>
        </product>
      </function>
      <function name="aero/moment/Pitch_alpha">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/cbarw-ft</property> <property>aero/alpha-rad</property>
          <value> {cm_a} </value>
        </product>
      </function>
      <function name="aero/moment/Pitch_damp">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/cbarw-ft</property> <property>aero/ci2vel</property>
          <property>velocities/q-aero-rad_sec</property> <value> -9.0 </value>
        </product>
      </function>
      <function name="aero/moment/Pitch_elevator">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/cbarw-ft</property> <property>fcs/elevator-pos-norm</property>
          <value> {cm_ele} </value>
        </product>
      </function>
      <function name="aero/moment/Pitch_elevator_propwash">
        <product>
          <property>fcs/throttle-cmd-norm</property>
          <property>fcs/elevator-pos-norm</property>
          <value> {pw_ele} </value>
        </product>
      </function>{flap_pitch}
    </axis>
    <axis name="YAW">
      <function name="aero/moment/Yaw_beta">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>aero/beta-rad</property>
          <value> 0.07 </value>
        </product>
      </function>
      <function name="aero/moment/Yaw_damp">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>aero/bi2vel</property>
          <property>velocities/r-aero-rad_sec</property> <value> -0.15 </value>
        </product>
      </function>
      <function name="aero/moment/Yaw_rudder">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/bw-ft</property> <property>fcs/rudder-pos-norm</property>
          <value> {yaw_rud} </value>
        </product>
      </function>
      <function name="aero/moment/Yaw_rudder_propwash">
        <product>
          <property>fcs/throttle-cmd-norm</property>
          <property>fcs/rudder-pos-norm</property>
          <value> {pw_rud} </value>
        </product>
      </function>
    </axis>
  </aerodynamics>
</fdm_config>
"""


FLAP_LIFT = """
      <function name="aero/force/Lift_flap">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>fcs/flap-cmd-norm</property> <value> {flap_cl} </value>
        </product>
      </function>"""
FLAP_DRAG = """
      <function name="aero/force/Drag_flap">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>fcs/flap-cmd-norm</property> <value> {flap_cd:.3f} </value>
        </product>
      </function>"""
FLAP_PITCH = """
      <function name="aero/moment/Pitch_flap">
        <product>
          <property>aero/qbar-psf</property> <property>metrics/Sw-sqft</property>
          <property>metrics/cbarw-ft</property>
          <property>fcs/flap-cmd-norm</property> <value> {flap_cm:.3f} </value>
        </product>
      </function>"""


def gen(name, p):
    span, area = p["span"], p["area"]
    chord = area / span
    auw = p["m_empty"] + p["m_batt"]
    thrust_kg = p["thrust_lbf"] * 4.448 / 9.81
    a_max = math.radians(p["a_max_deg"])
    slope = (p["clmax"] - p["cl0"]) / a_max
    vals = dict(
        name=name, desc=p["desc"], span=span, area=area, chord=chord,
        htail=0.18 * area, vtail=0.10 * area, arm=0.45 * span * 0.6,
        m_empty=p["m_empty"], m_batt=p["m_batt"], auw=auw,
        thrust_kg=thrust_kg, tw=thrust_kg / auw,
        ixx=0.06 * auw * (span / 2) ** 2,
        iyy=0.10 * auw * (span / 2) ** 2 * p["iyy_scale"],
        izz=0.15 * auw * (span / 2) ** 2,
        thrust_lbf=p["thrust_lbf"], thr_x=p["thr_xz"][0],
        thr_z=p["thr_xz"][1], torque_lbf=p["torque_lbf"],
        cl0=p["cl0"], clmax=p["clmax"], a_max=a_max,
        a_half=a_max / 2, cl_half=p["cl0"] + slope * a_max / 2,
        cl_post=0.80 * p["clmax"], cl_post2=0.65 * p["clmax"],
        clmin=p["clmin"], clmin_half=0.55 * p["clmin"],
        clmin_brk=0.75 * p["clmin"],
        cd0=p["cd0"], k_ind=p["k_ind"], cm0=p["cm0"], cm_a=p["cm_a"],
        cm_ele=p["cm_ele"], roll_ail=p["roll_ail"], yaw_rud=p["yaw_rud"],
        pw_ele=p["pw_ele"], pw_rud=p["pw_rud"], dihedral=p["dihedral"],
        flap_lift="", flap_drag="", flap_pitch="")
    # plain (unblown) landing flaps where the airframe has them: lift and
    # drag increments plus the nose-down trim change, all proportional to
    # fcs/flap-cmd-norm (the plant slews that property like a flap servo)
    if p.get("flap_cl", 0):
        vals["flap_lift"] = FLAP_LIFT.format(flap_cl=p["flap_cl"])
        vals["flap_drag"] = FLAP_DRAG.format(flap_cd=0.14 * p["flap_cl"])
        vals["flap_pitch"] = FLAP_PITCH.format(flap_cm=-0.20 * p["flap_cl"])
    d = os.path.join(HERE, "jsbsim", "aircraft", name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, f"{name}.xml"), "w", newline="\n") as fh:
        fh.write(TPL.format(**vals))
    vs = math.sqrt(2 * auw * 9.81 / (1.225 * area * p["clmax"]))
    print(f"{name:12s} AUW {auw:.2f} kg  T/W {thrust_kg / auw:.2f}  "
          f"stall {vs:4.1f} m/s  AR {span * span / area:.1f}")


if __name__ == "__main__":
    for name, p in AIRFRAMES.items():
        gen(name, p)
