"""K-wire task adapter built beside platform_a; platform_a clamp code is untouched."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from typing import Sequence
import numpy as np
from .handeye_calibration import pose_to_matrix
from .tool_center_calibration import flange_pose_for_center

@dataclass(frozen=True)
class KWireMotionConfig:
    needle_offset_mm: float = 40.0
    midpoint_to_grip_mm: float = 40.0
    z_plus_mm: float = 10.0
    pre_approach_mm: float = 60.0
    near_approach_mm: float = 20.0

def _v(x, name):
    a=np.asarray(x,dtype=float)
    if a.shape!=(3,) or not np.all(np.isfinite(a)): raise ValueError(f'{name} invalid')
    return a

def _unit(x):
    n=np.linalg.norm(x)
    if n<1e-10: raise ValueError('degenerate axis')
    return x/n

def tool_frame(wire_direction: Sequence[float], sole_normal: Sequence[float]) -> np.ndarray:
    """Semantic frame: Z || sole normal; Y is perpendicular to wire."""
    z=_unit(_v(sole_normal,'sole_normal')); v=_unit(_v(wire_direction,'wire_direction'))
    y0=np.cross(z,v)
    if np.linalg.norm(y0)<1e-10:
        ref=np.array([1.,0.,0.]) if abs(z[0])<.9 else np.array([0.,1.,0.]); y0=np.cross(z,ref)
    y=_unit(y0); x=_unit(np.cross(y,z)); return np.column_stack((x,y,z))

def build_plan(vision: dict, observation_flange_pose_mm_deg: Sequence[float], tcp_offset_mm: Sequence[float], cfg: KWireMotionConfig|None=None) -> dict:
    cfg=cfg or KWireMotionConfig();
    if not vision.get('valid',True): raise ValueError('vision target is not valid')
    midpoint=np.asarray(vision.get('midpoint_base_mm') or vision.get('clamp_contact_center_base_mm'),float)
    if midpoint.shape!=(3,): raise ValueError('vision needs midpoint_base_mm')
    midpoint/=1000.0
    v=_unit(_v(vision['wire_direction_base'],'wire_direction_base')); n=_unit(_v(vision['sole_normal_base'],'sole_normal_base'))
    R=tool_frame(v,n); approach=_unit(_v(vision.get('approach_axis_base',R[:,2]),'approach_axis_base'))
    rpy=vision.get('target_rpy_deg')
    if rpy is None: rpy=[float(x) for x in observation_flange_pose_mm_deg[3:]]
    # The visual midpoint is approached along the existing platform_a style axis.
    centers={'align':midpoint-approach*min(0.35,cfg.pre_approach_mm/1000.0+0.10),'pre':midpoint-approach*cfg.pre_approach_mm/1000.0,'near':midpoint-approach*cfg.near_approach_mm/1000.0,'midpoint':midpoint}
    corrected_y=midpoint-R[:,1]*(cfg.midpoint_to_grip_mm+cfg.needle_offset_mm)/1000.0
    final=corrected_y+R[:,2]*cfg.z_plus_mm/1000.0; centers['tool_y_minus']=corrected_y; centers['tool_z_plus']=final
    stages={}
    for name,p in centers.items():
        flange=flange_pose_for_center(p*1000.0,rpy,tcp_offset_mm); stages[name]={'center_base_mm':(p*1000).tolist(),'flange_pose_mm_deg':flange}
    return {'valid':True,'task':'k_wire','config':asdict(cfg),'midpoint_base_mm':(midpoint*1000).tolist(),'tool_frame_base':R.tolist(),'wire_direction_base':v.tolist(),'sole_normal_base':n.tolist(),'stages':stages,'final_center_base_mm':(final*1000).tolist(),'motion_allowed':False}

def unit_test():
    v=np.array([1.,0.,0.]); n=np.array([0.,0.,1.]); R=tool_frame(v,n); assert np.allclose(R,np.eye(3)); p=build_plan({'midpoint_base_mm':[0,0,0],'wire_direction_base':v.tolist(),'sole_normal_base':n.tolist()},[0,0,0,0,0,0],[0,0,0]); assert np.allclose(p['final_center_base_mm'],[0,-80,10]); assert np.max(abs(R.T@R-np.eye(3)))<1e-12; print('k_wire_motion unit test PASS')
