#!/usr/bin/env python3
"""Generates the JSBSim A320 model used by the trainer (data/aircraft/a320/a320.xml).

The model is built from publicly available A320-200 geometry, weights and typical
performance figures (stall speeds per configuration, L/D, approach thrust), not from
manufacturer data. Fly-by-wire control laws live in wasm/bridge.cpp; this file only
defines the bare airframe, so the FCS here maps commands straight to surfaces.

Structural frame: x aft from the nose, y right, z up from the fuselage centerline (inches).
"""
import math
import os

M = 39.3701  # inches per meter
DEG = math.pi / 180

# Geometry (meters)
CG_X = 16.40
NOSE_GEAR_X, MAIN_GEAR_X = 5.07, 17.71
MAIN_TRACK = 7.59
GEAR_Z = -4.15  # wheel bottom (uncompressed); ~0.3 m static stroke brings it to -3.85
CG_Z = -0.50
ENGINE_X, ENGINE_Y, ENGINE_Z = 14.0, 5.75, -2.15

# Aerodynamics
S_FT2, SPAN_FT, MAC_FT = 1319.7, 111.9, 14.07
CONFS = [0, 1, 2, 3, 4]  # lever positions 0, 1+F, 2, 3, FULL
CL0 = [0.22, 0.55, 0.80, 0.95, 1.25]
CLA_DEG = [0.087, 0.090, 0.093, 0.094, 0.096]
CLMAX = [1.50, 2.15, 2.45, 2.60, 2.80]
CD0_CONF = [0.0, 0.012, 0.024, 0.034, 0.062]
CM0_CONF = [0.03, -0.01, -0.06, -0.09, -0.14]
FLAP_DEG = [0, 10, 15, 20, 35]
SLAT_DEG = [0, 18, 22, 22, 27]


def stall_alpha(i):
    return (CLMAX[i] - CL0[i]) / CLA_DEG[i]


def cl_curve(i, a_deg):
    """Lift coefficient vs body alpha with a rounded peak and post-stall drop."""
    a_st = stall_alpha(i)
    lin = CL0[i] + CLA_DEG[i] * a_deg
    if a_deg <= a_st - 3:
        return lin
    if a_deg <= a_st:  # round off the top of the curve
        t = (a_deg - (a_st - 3)) / 3
        top = CLMAX[i]
        start = CL0[i] + CLA_DEG[i] * (a_st - 3)
        return start + (top - start) * (1 - (1 - t) ** 2)
    if a_deg <= a_st + 6:  # stall break
        t = (a_deg - a_st) / 6
        return CLMAX[i] * (1 - 0.30 * t)
    return CLMAX[i] * 0.70 * max(0.0, math.cos((a_deg - a_st - 6) * DEG * 1.5))


def table2d(row_prop, col_prop, rows, cols, fn, fmt='{:.4f}'):
    head = '\t'.join(str(c) for c in cols)
    lines = [f'          \t{head}']
    for r in rows:
        lines.append(f'          {r:.4f}\t' + '\t'.join(fmt.format(fn(r, c)) for c in cols))
    body = '\n'.join(lines)
    return f'''<table>
        <independentVar lookup="row">{row_prop}</independentVar>
        <independentVar lookup="column">{col_prop}</independentVar>
        <tableData>
{body}
        </tableData>
      </table>'''


def table1d(prop, pairs):
    body = '\n'.join(f'          {a:.4f}\t{b:.4f}' for a, b in pairs)
    return f'''<table>
        <independentVar>{prop}</independentVar>
        <tableData>
{body}
        </tableData>
      </table>'''


def coef(name, desc, *factors, table=None, value=None):
    props = '\n'.join(f'      <property>{p}</property>' for p in factors)
    extra = ''
    if table:
        extra += '\n      ' + table
    if value is not None:
        extra += f'\n      <value>{value}</value>'
    return f'''  <function name="aero/coefficient/{name}">
    <description>{desc}</description>
    <product>
{props}{extra}
    </product>
  </function>'''


def loc(x, y, z):
    return f'<location unit="IN"> <x> {x * M:.1f} </x> <y> {y * M:.1f} </y> <z> {z * M:.1f} </z> </location>'


def gear(name, x, y, k, c, c_reb, steer, brake):
    return f'''    <contact type="BOGEY" name="{name}">
      {loc(x, y, GEAR_Z)}
      <static_friction> 0.80 </static_friction>
      <dynamic_friction> 0.50 </dynamic_friction>
      <rolling_friction> 0.015 </rolling_friction>
      <spring_coeff unit="LBS/FT"> {k} </spring_coeff>
      <damping_coeff unit="LBS/FT/SEC"> {c} </damping_coeff>
      <damping_coeff_rebound unit="LBS/FT/SEC"> {c_reb} </damping_coeff_rebound>
      <max_steer unit="DEG"> {steer} </max_steer>
      <brake_group> {brake} </brake_group>
      <retractable> 1 </retractable>
    </contact>'''


def structure(name, x, y, z):
    return f'''    <contact type="STRUCTURE" name="{name}">
      {loc(x, y, z)}
      <static_friction> 1.0 </static_friction>
      <dynamic_friction> 0.8 </dynamic_friction>
      <spring_coeff unit="LBS/FT"> 300000 </spring_coeff>
      <damping_coeff unit="LBS/FT/SEC"> 50000 </damping_coeff>
    </contact>'''


def kinematic(name, inp, out, settings):
    s = '\n'.join(f'          <setting> <position> {p} </position> <time> {t} </time> </setting>' for p, t in settings)
    return f'''      <kinematic name="{name}">
        <input>{inp}</input>
        <traverse>
{s}
        </traverse>
        <output>{out}</output>
      </kinematic>'''


def surface(name, inp, out, max_rad):
    return f'''      <aerosurface_scale name="{name}">
        <input>{inp}</input>
        <range> <min>{-max_rad:.4f}</min> <max>{max_rad:.4f}</max> </range>
        <output>{out}</output>
      </aerosurface_scale>'''


def conf_table(prop_values):
    body = '\n'.join(f'          {c} {v}' for c, v in zip(CONFS, prop_values))
    return f'<table> <independentVar>fcs/conf-pos</independentVar> <tableData>\n{body}\n        </tableData> </table>'


alphas_deg = list(range(-10, 41, 1))
alphas_rad = [a * DEG for a in alphas_deg]

QS = ('aero/qbar-psf', 'metrics/Sw-sqft')
QSB = QS + ('metrics/bw-ft',)
QSC = QS + ('metrics/cbarw-ft',)

aero = []
# ---------------------------------------------------------------- LIFT
aero.append('<axis name="LIFT">')
aero.append(coef('CLbasic', 'Lift vs alpha and configuration', *QS,
                 table=table2d('aero/alpha-rad', 'fcs/conf-pos', alphas_rad, CONFS,
                               lambda a, c: cl_curve(c, a / DEG))))
aero.append(coef('CLmach', 'Compressibility increase of the lift slope (clean)', *QS, 'aero/alpha-rad',
                 table=table2d('velocities/mach', 'fcs/conf-pos',
                               [0.0, 0.4, 0.6, 0.7, 0.78, 0.85, 0.95], [0, 1],
                               lambda m, c: (0 if c else 1) * {0.0: 0, 0.4: 0.45, 0.6: 1.25, 0.7: 2.0,
                                                               0.78: 2.75, 0.85: 2.6, 0.95: 1.5}[m])))
aero.append(coef('CLge', 'Ground effect lift increase', 'aero/coefficient/CLbasic',
                 table=table1d('aero/h_b-mac-ft', [(0, 0.12), (0.1, 0.07), (0.2, 0.04), (0.4, 0.01), (0.6, 0)])))
aero.append(coef('CLde', 'Lift due to elevator', *QS, 'fcs/elevator-pos-rad', value=0.30))
aero.append(coef('CLq', 'Lift due to pitch rate', *QS, 'aero/ci2vel', 'velocities/q-aero-rad_sec', value=5.0))
aero.append(coef('CLsb', 'Lift loss from speedbrakes', *QS, 'fcs/speedbrake-pos-norm', value=-0.15))
aero.append(coef('CLsp', 'Lift dump from ground spoilers', *QS, 'fcs/spoiler-pos-norm', value=-0.85))
aero.append('</axis>')

# ---------------------------------------------------------------- DRAG
aero.append('<axis name="DRAG">')
aero.append(coef('CD0', 'Parasite drag', *QS, value=0.0190))
aero.append(coef('CDconf', 'Drag from slats and flaps', *QS, table=conf_table(CD0_CONF)))
aero.append(coef('CDi', 'Induced drag', *QS, 'aero/cl-squared', value=0.045))
aero.append(coef('CDge', 'Ground effect induced drag reduction', 'aero/coefficient/CDi',
                 table=table1d('aero/h_b-mac-ft', [(0, -0.45), (0.1, -0.30), (0.2, -0.18), (0.4, -0.06), (0.6, 0)])))
aero.append(coef('CDmach', 'Wave drag', *QS,
                 table=table1d('velocities/mach', [(0.0, 0), (0.72, 0), (0.75, 0.0005), (0.78, 0.0015),
                                                   (0.80, 0.003), (0.82, 0.006), (0.85, 0.016), (0.90, 0.04), (1.0, 0.09)])))
aero.append(coef('CDstall', 'Drag rise beyond the stall', *QS,
                 table=table1d('aero/alpha-rad', [(-1.0, 0), (14 * DEG, 0), (20 * DEG, 0.15), (30 * DEG, 0.5), (45 * DEG, 1.0)])))
aero.append(coef('CDgear', 'Landing gear drag', *QS, 'gear/gear-pos-norm', value=0.020))
aero.append(coef('CDsb', 'Speedbrake drag', *QS, 'fcs/speedbrake-pos-norm', value=0.030))
aero.append(coef('CDsp', 'Ground spoiler drag', *QS, 'fcs/spoiler-pos-norm', value=0.080))
aero.append(coef('CDbeta', 'Drag due to sideslip', *QS, 'aero/mag-beta-rad', value=0.30))
aero.append(coef('CDde', 'Drag due to elevator', *QS, 'fcs/mag-elevator-pos-rad', value=0.03))
aero.append('</axis>')

# ---------------------------------------------------------------- SIDE
aero.append('<axis name="SIDE">')
aero.append(coef('CYb', 'Side force due to sideslip', *QS, 'aero/beta-rad', value=-0.90))
aero.append(coef('CYdr', 'Side force due to rudder', *QS, 'fcs/rudder-pos-rad', value=0.25))
aero.append('</axis>')

# ---------------------------------------------------------------- ROLL
aero.append('<axis name="ROLL">')
aero.append(coef('Clb', 'Roll due to sideslip (dihedral)', *QSB, 'aero/beta-rad', value=-0.10))
aero.append(coef('Clp', 'Roll damping', *QSB, 'aero/bi2vel', 'velocities/p-aero-rad_sec', value=-0.45))
aero.append(coef('Clr', 'Roll due to yaw rate', *QSB, 'aero/bi2vel', 'velocities/r-aero-rad_sec', value=0.15))
aero.append(coef('Clda', 'Roll due to ailerons and roll spoilers', *QSB, 'fcs/left-aileron-pos-rad', value=0.11))
aero.append(coef('Cldr', 'Roll due to rudder', *QSB, 'fcs/rudder-pos-rad', value=0.012))
aero.append('</axis>')

# ---------------------------------------------------------------- PITCH
aero.append('<axis name="PITCH">')
aero.append(coef('Cm0', 'Pitching moment at zero alpha', *QSC, table=conf_table(CM0_CONF)))
aero.append(coef('Cma', 'Pitch stability', *QSC, 'aero/alpha-rad', value=-1.20))
aero.append(coef('Cmstall', 'Nose-down pitch break beyond the stall', *QSC,
                 table=table1d('aero/alpha-rad', [(-1.0, 0), (16 * DEG, 0), (20 * DEG, -0.10), (30 * DEG, -0.30), (45 * DEG, -0.45)])))
aero.append(coef('Cmq', 'Pitch damping', *QSC, 'aero/ci2vel', 'velocities/q-aero-rad_sec', value=-22.0))
aero.append(coef('Cmadot', 'Pitch due to alpha rate', *QSC, 'aero/ci2vel', 'aero/alphadot-rad_sec', value=-8.0))
aero.append(coef('Cmde', 'Pitch due to elevator', *QSC, 'fcs/elevator-pos-rad', value=-1.30))
aero.append(coef('Cmge', 'Nose-down moment in ground effect', *QSC,
                 table=table1d('aero/h_b-mac-ft', [(0, -0.04), (0.2, -0.015), (0.5, 0)])))
aero.append('</axis>')

# ---------------------------------------------------------------- YAW
aero.append('<axis name="YAW">')
aero.append(coef('Cnb', 'Weathercock stability', *QSB, 'aero/beta-rad', value=0.13))
aero.append(coef('Cnr', 'Yaw damping', *QSB, 'aero/bi2vel', 'velocities/r-aero-rad_sec', value=-0.28))
aero.append(coef('Cnp', 'Yaw due to roll rate', *QSB, 'aero/bi2vel', 'velocities/p-aero-rad_sec', value=-0.04))
aero.append(coef('Cndr', 'Yaw due to rudder', *QSB, 'fcs/rudder-pos-rad', value=-0.10))
aero.append(coef('Cnda', 'Adverse yaw', *QSB, 'fcs/left-aileron-pos-rad', value=-0.004))
aero.append('</axis>')

aero_xml = '\n'.join(aero)

contacts = '\n'.join([
    # oleo damping sized so a 10 ft/s touchdown adds roughly 1 g
    gear('NOSE_LG', NOSE_GEAR_X, 0, 18000, 1500, 5000, 75, 'NONE'),
    gear('LEFT_MLG', MAIN_GEAR_X, -MAIN_TRACK / 2, 65000, 6000, 20000, 0, 'LEFT'),
    gear('RIGHT_MLG', MAIN_GEAR_X, MAIN_TRACK / 2, 65000, 6000, 20000, 0, 'RIGHT'),
    structure('TAIL', 32.5, 0, -0.62),
    structure('LEFT_ENGINE', ENGINE_X - 1.0, -ENGINE_Y, -3.34),
    structure('RIGHT_ENGINE', ENGINE_X - 1.0, ENGINE_Y, -3.34),
    structure('LEFT_WINGTIP', 20.5, -17.05, 0.30),
    structure('RIGHT_WINGTIP', 20.5, 17.05, 0.30),
    structure('NOSE', 0.4, 0, -1.2),
    structure('BELLY', 16.5, 0, -1.98),
])


def engine(i, side):
    return f'''    <engine file="CFM56_5B4">
      <feed>{i}</feed>
      <thruster file="direct">
        {loc(ENGINE_X, side * ENGINE_Y, ENGINE_Z)}
        <orient unit="DEG"> <roll> 0 </roll> <pitch> 0 </pitch> <yaw> 0 </yaw> </orient>
      </thruster>
    </engine>'''


def tank(side, contents):
    return f'''    <tank type="FUEL">
      {loc(CG_X - 0.3, side * 4.0, -1.0)}
      <capacity unit="LBS"> 15000 </capacity>
      <contents unit="LBS"> {contents} </contents>
    </tank>'''


xml = f'''<?xml version="1.0"?>
<!-- Generated by gen_a320.py - edit the generator, not this file. -->
<fdm_config name="A320-200 trainer" version="2.0" release="BETA">
  <fileheader>
    <author>Sky Hopper</author>
    <filecreationdate>2026-10-06</filecreationdate>
    <version>1.0</version>
    <description>Airbus A320-200 (CFM56-5B) for training use, built from public data</description>
    <note>Not validated against manufacturer data. For training familiarisation only.</note>
  </fileheader>

  <metrics>
    <wingarea unit="FT2"> {S_FT2} </wingarea>
    <wingspan unit="FT"> {SPAN_FT} </wingspan>
    <chord unit="FT"> {MAC_FT} </chord>
    <htailarea unit="FT2"> 333.6 </htailarea>
    <htailarm unit="FT"> 44.4 </htailarm>
    <vtailarea unit="FT2"> 231.3 </vtailarea>
    <vtailarm unit="FT"> 41.0 </vtailarm>
    <location name="AERORP" unit="IN"> <x> {CG_X * M:.1f} </x> <y> 0 </y> <z> {CG_Z * M:.1f} </z> </location>
    <location name="EYEPOINT" unit="IN"> <x> {3.0 * M:.1f} </x> <y> {-0.55 * M:.1f} </y> <z> {0.85 * M:.1f} </z> </location>
    <location name="VRP" unit="IN"> <x> {CG_X * M:.1f} </x> <y> 0 </y> <z> {CG_Z * M:.1f} </z> </location>
  </metrics>

  <mass_balance negated_crossproduct_inertia="true">
    <ixx unit="SLUG*FT2"> 942877 </ixx>
    <iyy unit="SLUG*FT2"> 2788920 </iyy>
    <izz unit="SLUG*FT2"> 3597570 </izz>
    <ixz unit="SLUG*FT2"> -10000 </ixz>
    <emptywt unit="LBS"> 93900 </emptywt>
    <location name="CG" unit="IN"> <x> {CG_X * M:.1f} </x> <y> 0 </y> <z> {CG_Z * M:.1f} </z> </location>
    <pointmass name="Payload">
      <weight unit="LBS"> 30000 </weight>
      <location name="POINTMASS" unit="IN"> <x> {CG_X * M:.1f} </x> <y> 0 </y> <z> 0 </z> </location>
    </pointmass>
  </mass_balance>

  <ground_reactions>
{contacts}
  </ground_reactions>

  <propulsion>
{engine(0, -1)}
{engine(1, 1)}
{tank(-1, 6500)}
{tank(1, 6500)}
  </propulsion>

  <flight_control name="A320 surfaces (laws are in the flight control computer)">
    <channel name="Pitch">
      <summer name="Pitch trim sum">
        <input>fcs/elevator-cmd-norm</input>
        <input>fcs/pitch-trim-cmd-norm</input>
        <clipto> <min>-1</min> <max>1</max> </clipto>
      </summer>
{surface('Elevator', 'fcs/pitch-trim-sum', 'fcs/elevator-pos-rad', 30 * DEG)}
    </channel>
    <channel name="Roll">
{surface('Left aileron', 'fcs/aileron-cmd-norm', 'fcs/left-aileron-pos-rad', 25 * DEG)}
{surface('Right aileron', '-fcs/aileron-cmd-norm', 'fcs/right-aileron-pos-rad', 25 * DEG)}
    </channel>
    <channel name="Yaw">
{surface('Rudder', 'fcs/rudder-cmd-norm', 'fcs/rudder-pos-rad', 30 * DEG)}
    </channel>
    <channel name="High lift">
{kinematic('Slats and flaps', 'fcs/flap-cmd-norm', 'fcs/conf-pos', [(0, 0), (1, 7), (2, 6), (3, 5), (4, 7)])}
      <fcs_function name="Flap angle">
        <function>{conf_table(FLAP_DEG)}</function>
        <output>fcs/flap-pos-deg</output>
      </fcs_function>
      <fcs_function name="Slat angle">
        <function>{conf_table(SLAT_DEG)}</function>
        <output>fcs/slat-pos-deg</output>
      </fcs_function>
    </channel>
    <channel name="Gear and spoilers">
{kinematic('Gear', 'gear/gear-cmd-norm', 'gear/gear-pos-norm', [(0, 0), (1, 10)])}
{kinematic('Speedbrakes', 'fcs/speedbrake-cmd-norm', 'fcs/speedbrake-pos-norm', [(0, 0), (1, 2)])}
{kinematic('Ground spoilers', 'fcs/spoiler-cmd-norm', 'fcs/spoiler-pos-norm', [(0, 0), (1, 1)])}
    </channel>
  </flight_control>

  <aerodynamics>
{aero_xml}
  </aerodynamics>
</fdm_config>
'''

here = os.path.dirname(os.path.abspath(__file__))
out_dir = os.path.join(here, '..', 'data', 'aircraft', 'a320')
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, 'a320.xml'), 'w') as f:
    f.write(xml)

# Engine: CFM56-5B4 (27,000 lbf), thrust tables from JSBSim's CFM56_5
src = os.path.join(here, '..', 'jsbsim', 'engine', 'CFM56_5.xml')
with open(src) as f:
    eng = f.read()
eng = eng.replace('<turbine_engine name="CFM56_5">', '<turbine_engine name="CFM56_5B4">')
eng = eng.replace('<milthrust> 25000.0 </milthrust>', '<milthrust> 27000.0 </milthrust>')
with open(os.path.join(here, '..', 'data', 'engine', 'CFM56_5B4.xml'), 'w') as f:
    f.write(eng)

for i in CONFS:
    print(f'conf {i}: stall alpha {stall_alpha(i):.1f} deg, CLmax {CLMAX[i]}')
