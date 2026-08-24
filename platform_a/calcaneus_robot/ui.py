from __future__ import annotations

import tkinter as tk
import os
import base64
import subprocess
import threading
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

from .controller import RobotController
from .device import make_adapter
from .models import ControlParameters, ControlState, PatientCase
from .storage import RecordStore


class CalcaneusRobotApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.withdraw()
        # WSLg 的系统标题栏缺少完整中文字形，中文标题会显示成方框编码。
        # 窗口内部仍保留原始中文软件名称，仅系统标题使用英文。
        self.title("Calcaneus Reduction Robot Control Software V1.0")
        self.geometry("1280x780"); self.minsize(1100, 700)
        self.option_add("*Font", ("Microsoft YaHei UI", 10))
        root = Path(__file__).resolve().parents[1]
        self.store = RecordStore(root / "data")
        device_mode = os.environ.get("PLATFORM_A_DEVICE_MODE", "real")
        self.controller = RobotController(
            make_adapter(device_mode), self.store, self.notify
        )
        self.vars = {k: tk.StringVar(value="0.00") for k in ("fx","fy","fz","tx","ty","tz","force","x","y","z","opening")}
        self.state_var = tk.StringVar(value=ControlState.DISCONNECTED.value)
        self.progress_var = tk.DoubleVar(value=0)
        self.camera_photo = None
        self.camera_frame_bytes = None
        self.operation_running = False
        self.active_branch = None
        if self.login_dialog():
            self._build()
            if self.controller.device.read_only:
                for button in self.control_buttons:
                    button.configure(state="disabled")
                # 保持原界面，只开放视觉预检查和现有“软件回零”按钮。
                self.clamp_button.configure(state="normal")
                self.control_buttons[0].configure(state="normal")
                self.home_button.configure(state="disabled")
                for index in (2, 3, 4, 6, 7, 8, 9):
                    self.control_buttons[index].configure(state="normal")
            self.deiconify(); self.after(100, self.refresh)
            self.bind("<Control-Shift-H>", lambda _event: self.record_home_position())
        else:
            self.destroy()

    def login_dialog(self) -> bool:
        dialog = tk.Toplevel(self); dialog.title("用户登录"); dialog.geometry("360x210"); dialog.resizable(False, False)
        ok = {"value": False}; user=tk.StringVar(value="operator"); pwd=tk.StringVar(value="demo123")
        ttk.Label(dialog,text="操作员账号").pack(pady=(22,4)); ttk.Entry(dialog,textvariable=user).pack(fill="x",padx=55)
        ttk.Label(dialog,text="密码").pack(pady=(9,4)); ttk.Entry(dialog,textvariable=pwd,show="*").pack(fill="x",padx=55)
        def submit():
            if user.get()=="operator" and pwd.get()=="demo123": ok["value"]=True; dialog.destroy()
            else: messagebox.showerror("登录失败","账号或密码错误",parent=dialog)
        ttk.Button(dialog,text="登录",command=submit).pack(pady=15); dialog.protocol("WM_DELETE_WINDOW",dialog.destroy)
        # WSLg maps Toplevel asynchronously; wait until it is viewable before
        # making it modal. This does not alter any visual/robot logic.
        dialog.update_idletasks(); dialog.wait_visibility(); dialog.grab_set(); dialog.focus_force()
        self.wait_window(dialog); return ok["value"]

    def _build(self) -> None:
        style=ttk.Style(); style.configure("Alarm.TButton",foreground="#b00020",font=("Microsoft YaHei UI",11,"bold"))
        top=ttk.Frame(self,padding=8); top.pack(fill="x")
        ttk.Label(top,text="跟骨撬拨夹挤微创复位辅助机器人控制软件",font=("Microsoft YaHei UI",16,"bold")).pack(side="left")
        ttk.Label(top,textvariable=self.state_var,foreground="#075a9c",font=("Microsoft YaHei UI",12,"bold")).pack(side="right",padx=12)
        ttk.Button(top,text="连接设备",command=lambda:self.safe(self.controller.connect)).pack(side="right",padx=3)
        ttk.Button(top,text="断开",command=lambda:self.safe(self.controller.disconnect)).pack(side="right",padx=3)
        body=ttk.Panedwindow(self,orient="horizontal"); body.pack(fill="both",expand=True,padx=8,pady=(0,8))
        left=ttk.Frame(body); center=ttk.Frame(body); right=ttk.Frame(body); body.add(left,weight=2); body.add(center,weight=3); body.add(right,weight=2)
        self._case_panel(left); self._parameter_panel(left); self._control_panel(left)
        self._visual_panel(center); self._wrench_panel(center)
        self._status_panel(right); self._log_panel(right)

    def group(self,parent,title):
        f=ttk.LabelFrame(parent,text=title,padding=8); f.pack(fill="x",padx=4,pady=4); return f

    def _case_panel(self,parent):
        f=self.group(parent,"病例与任务"); self.case_entries={}
        defaults={"病例编号":f"CASE-{datetime.now():%Y%m%d-%H%M}","患者编码":"P001","操作者":"operator"}
        for i,(name,val) in enumerate(defaults.items()):
            ttk.Label(f,text=name).grid(row=i,column=0,sticky="w",pady=2); v=tk.StringVar(value=val); ttk.Entry(f,textvariable=v,width=22).grid(row=i,column=1,sticky="ew"); self.case_entries[name]=v
        ttk.Label(f,text="患侧").grid(row=3,column=0,sticky="w"); self.side=tk.StringVar(value="左"); ttk.Combobox(f,textvariable=self.side,values=("左","右"),state="readonly",width=19).grid(row=3,column=1)
        ttk.Button(f,text="建立病例",command=self.create_case).grid(row=4,column=0,columnspan=2,sticky="ew",pady=(7,0)); f.columnconfigure(1,weight=1)

    def _parameter_panel(self,parent):
        f=self.group(parent,"规划与安全参数"); self.param_entries={}
        items=(("撬拨位移(mm)",50.0),("夹挤位移(mm)",5.0),("速度(mm/s)",20.0),("力上限(N)",80.0),("力矩上限(Nm)",8.0),("保持时间(s)",3.0))
        for i,(name,val) in enumerate(items):
            ttk.Label(f,text=name).grid(row=i,column=0,sticky="w",pady=2); v=tk.StringVar(value=str(val)); ttk.Entry(f,textvariable=v,width=12).grid(row=i,column=1); self.param_entries[name]=v
        ttk.Button(f,text="校验并应用参数",command=self.apply_params).grid(row=len(items),column=0,columnspan=2,sticky="ew",pady=(7,0))

        # Independent pry trajectory controls; clamp branch does not read these.
        list(self.param_entries.values())[0].set("100.0")
        self.pry_direction_var = tk.StringVar(value="X_PLUS")
        self.pry_angle_var = tk.StringVar(value="45")
        ttk.Label(f, text="撬拨方向").grid(row=len(items)+1, column=0, sticky="w", pady=2)
        ttk.Combobox(f, textvariable=self.pry_direction_var, values=("X_PLUS", "X_MINUS", "Y_PLUS", "Y_MINUS"), state="readonly", width=12).grid(row=len(items)+1, column=1)
        ttk.Label(f, text="撬拨角度(deg)").grid(row=len(items)+2, column=0, sticky="w", pady=2)
        ttk.Entry(f, textvariable=self.pry_angle_var, width=12).grid(row=len(items)+2, column=1)

    def _control_panel(self,parent):
        f=self.group(parent,"复位辅助控制")
        buttons=(("开始撬拨定位",self.enter_pry_branch),("开始夹挤复位",self.start_clamp),("暂停",self.controller.pause),("继续",self.controller.resume),("安全停止",self.stop_task),("软件回零",self.start_home),("夹爪张开 +1mm",lambda:self.controller.jog_gripper(1)),("夹爪闭合 -1mm",lambda:self.controller.jog_gripper(-1)))
        self.control_buttons=[]
        for i,(text,cmd) in enumerate(buttons):
            button=ttk.Button(f,text=text,command=lambda c=cmd:self.safe(c))
            button.grid(row=i//2,column=i%2,sticky="ew",padx=2,pady=2)
            self.control_buttons.append(button)
        self.clamp_button=self.control_buttons[1]
        self.home_button=self.control_buttons[5]
        self.pry_execute_button=ttk.Button(f,text="启动撬拨",command=lambda:self.safe(self.start_pry_execution))
        self.pry_execute_button.grid(row=5,column=0,sticky="ew",padx=2,pady=2)
        self.clamp_execute_button=ttk.Button(f,text="启动夹挤",command=lambda:self.safe(self.execute_clamp))
        self.clamp_execute_button.grid(row=5,column=1,sticky="ew",padx=2,pady=2)
        # 不改变原界面布局：右键“软件回零”或按 Ctrl+Shift+H 记录当前停稳位置为新零点。
        self.home_button.bind("<Button-3>", lambda _event: self.record_home_position())
        emergency=ttk.Button(f,text="急停",style="Alarm.TButton",command=lambda:self.safe(self.controller.emergency_stop))
        emergency.grid(row=4,column=0,sticky="ew",padx=2,pady=5)
        reset=ttk.Button(f,text="急停复位",command=lambda:self.safe(self.controller.reset_emergency))
        reset.grid(row=4,column=1,sticky="ew",padx=2,pady=5)
        self.control_buttons.extend((emergency,reset))
        for i in range(2): f.columnconfigure(i,weight=1)
        self.pry_execute_button.configure(state="disabled")
        self.clamp_execute_button.configure(state="disabled")

    def stop_task(self):
        self.controller.stop()
        # Safe stop ends the active branch selection, so the operator can
        # choose the other branch without restarting the application.
        self.operation_running = False
        self.active_branch = None
        self.pry_execute_button.configure(state="disabled")
        self.clamp_execute_button.configure(state="disabled")
        self.control_buttons[0].configure(state="normal")
        self.clamp_button.configure(state="normal")
        self.home_button.configure(state="normal")
        self.log("安全停止完成：撬拨和夹挤分支均可重新选择")

    def _visual_panel(self,parent):
        f=ttk.LabelFrame(parent,text="二维术野与机器人运动示意",padding=5); f.pack(fill="both",expand=True,padx=4,pady=4)
        self.canvas=tk.Canvas(f,bg="#eef3f6",height=360,highlightthickness=0); self.canvas.pack(fill="both",expand=True)
        self.canvas.bind("<Configure>",lambda e:self.draw_scene())

    def _wrench_panel(self,parent):
        f=self.group(parent,"六维力/力矩实时监测")
        for i,key in enumerate(("fx","fy","fz","tx","ty","tz","force")):
            label={"fx":"Fx (N)","fy":"Fy (N)","fz":"Fz (N)","tx":"Tx (Nm)","ty":"Ty (Nm)","tz":"Tz (Nm)","force":"合力 (N)"}[key]
            box=ttk.Frame(f); box.grid(row=i//4,column=i%4,padx=8,pady=4); ttk.Label(box,text=label).pack(); ttk.Label(box,textvariable=self.vars[key],font=("Consolas",13,"bold")).pack()

    def _status_panel(self,parent):
        f=self.group(parent,"位置与流程状态")
        real=self.controller.device.read_only
        labels=(
            {"x":"TCP X","y":"TCP Y","z":"TCP Z","opening":"夹爪位置"}
            if real else
            {"x":"夹挤行程 X","y":"横向位置 Y","z":"撬拨高度 Z","opening":"夹爪开度"}
        )
        for i,key in enumerate(("x","y","z","opening")):
            ttk.Label(f,text=labels[key]).grid(row=i,column=0,sticky="w",pady=3)
            ttk.Label(f,textvariable=self.vars[key],font=("Consolas",12,"bold")).grid(row=i,column=1,sticky="e")
        self.progress=ttk.Progressbar(f,variable=self.progress_var,maximum=100); self.progress.grid(row=4,column=0,columnspan=2,sticky="ew",pady=8)
        ttk.Button(f,text="导出过程记录",command=self.export).grid(row=5,column=0,columnspan=2,sticky="ew"); f.columnconfigure(1,weight=1)

    def _log_panel(self,parent):
        f=ttk.LabelFrame(parent,text="事件日志",padding=5); f.pack(fill="both",expand=True,padx=4,pady=4)
        self.log_text=tk.Text(f,height=18,state="disabled",wrap="word",font=("Consolas",9)); self.log_text.pack(fill="both",expand=True)

    def create_case(self):
        try:
            values={k:v.get().strip() for k,v in self.case_entries.items()}
            if not all(values.values()): raise ValueError("病例编号、患者编码和操作者不能为空")
            self.store.begin_case(PatientCase(values["病例编号"],values["患者编码"],self.side.get(),values["操作者"]))
            self.log("病例建立成功")
        except Exception as e: messagebox.showerror("建立失败",str(e),parent=self)

    def apply_params(self):
        try:
            v={k:float(x.get()) for k,x in self.param_entries.items()}
            p=ControlParameters(v["撬拨位移(mm)"],v["夹挤位移(mm)"],v["速度(mm/s)"],v["力上限(N)"],v["力矩上限(Nm)"],v["保持时间(s)"])
            self.controller.update_parameters(p); self.log("参数已应用")
            return True
        except Exception as e:
            messagebox.showerror("参数错误",str(e),parent=self)
            return False

    def safe(self,func):
        try: func()
        except Exception as e: self.log(f"操作拒绝：{e}"); messagebox.showwarning("操作未执行",str(e),parent=self)

    def notify(self, message):
        if threading.current_thread() is threading.main_thread():
            self.log(message)
        else:
            self.after(0, lambda m=message: self.log(m))

    def start_home(self):
        if not self.controller.device.read_only:
            self.safe(self.controller.home)
            return
        if self.operation_running:
            messagebox.showwarning("操作正在进行", "请等待当前动作完成。", parent=self)
            return
        if not messagebox.askyesno(
            "确认夹挤复位",
            "夹爪将先完全张开，机械臂随后沿安全路线返回初始位置。\n"
            "请确认足模型已固定、路径无人且急停可用。",
            parent=self,
        ):
            return
        self.operation_running = True
        self.home_button.configure(state="disabled")
        self.clamp_button.configure(state="disabled")
        self.log("安全回零命令已提交，请等待返回撬拨观察零点")

        def worker():
            error = None
            try:
                self.controller.home()
            except Exception as exc:
                error = exc

            def finish():
                self.operation_running = False
                self.home_button.configure(state="normal")
                self.clamp_button.configure(state="normal")
                if error is not None:
                    self.log(f"夹挤复位失败：{error}")
                    messagebox.showwarning("夹挤复位未完成", str(error), parent=self)
                else:
                    # Reset ends the previous branch. A subsequent operation
                    # must enter a fresh branch and start a fresh vision worker.
                    self.active_branch = None
                    self.control_buttons[0].configure(state="normal")
                    self.clamp_button.configure(state="normal")
                    self.pry_execute_button.configure(state="disabled")
                    self.clamp_execute_button.configure(state="disabled")
                    messagebox.showinfo(
                        "夹挤复位完成",
                        "夹爪已完全张开，机械臂已返回撬拨观察零点。",
                        parent=self,
                    )

            self.after(0, finish)

        threading.Thread(target=worker, name="platform-a-home", daemon=True).start()

    def record_home_position(self):
        """记录当前停稳位姿；不发送机器人运动命令。"""
        if not self.controller.device.read_only:
            messagebox.showwarning("当前为仿真模式", "请在真机只读模式下记录零点。", parent=self)
            return
        if self.operation_running:
            messagebox.showwarning("操作进行中", "请等待当前动作完成后再记录零点。", parent=self)
            return
        if not messagebox.askyesno(
            "记录软件零点",
            "请确认机械臂已经停稳、夹爪处于你希望的默认位置。\n\n保存后，今后点击软件回零会返回这个位置。",
            parent=self,
        ):
            return
        root = Path(__file__).resolve().parents[2]
        script = root / ("scripts/save_pry_home_position.py" if self.active_branch == "pry" else "scripts/save_home_position.py")
        try:
            result = subprocess.run(
                [os.environ.get("PYTHON", "python3"), str(script)],
                cwd=root,
                text=True,
                capture_output=True,
                timeout=15,
                check=False,
            )
            output = "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
            if result.returncode != 0:
                raise RuntimeError(output or "记录零点失败")
            self.log("已记录新的软件零点")
            messagebox.showinfo("记录成功", "新的软件零点已保存。以后软件回零将返回这个位置。", parent=self)
        except Exception as exc:
            messagebox.showwarning("记录失败", str(exc), parent=self)

    def enter_pry_branch(self):
        if self.active_branch is not None:
            return
        self.controller.start_positioning()
        self.active_branch = "pry"
        self.home_button.configure(state="normal")
        self.pry_execute_button.configure(state="normal")
        self.clamp_button.configure(state="disabled")
        self.log("已进入撬拨分支：仅显示视觉，尚未计算目标或运动")

    def start_pry_execution(self):
        result = dict(self.controller.device.pry_vision.result)
        result["pry_direction"] = self.pry_direction_var.get()
        result["pry_angle_deg"] = float(self.pry_angle_var.get())
        result["pry_lever_arm_mm"] = float(list(self.param_entries.values())[0].get())
        result["pry_speed_mm_s"] = float(self.param_entries["速度(mm/s)"].get())
        if not result.get("valid"):
            raise RuntimeError(result.get("message") or "撬拨视觉结果尚未满足50–60 mm校验")
        if not messagebox.askyesno(
            "确认移动到撬拨夹持点",
            f"夹持宽度 {float(result.get('width_mm')):.2f} mm。\n"
            "机械臂将保持夹爪张开，仅移动到夹持中心，不闭合夹爪。\n"
            "请确认现场无人、无障碍且实体急停可用。",
            parent=self,
        ):
            return
        self.operation_running = True
        self.pry_execute_button.configure(state="disabled")
        self.home_button.configure(state="disabled")
        self.log(f"撬拨启动：移动到夹持点，夹爪保持张开，宽度 {float(result.get('width_mm')):.2f} mm")
        def worker():
            error = None
            try:
                self.controller.device.start_pry_workflow(result)
            except Exception as exc:
                error = exc
            def finish():
                self.operation_running = False
                self.pry_execute_button.configure(state="normal")
                self.home_button.configure(state="normal")
                if error is not None:
                    if self.controller.state == ControlState.POSITIONING:
                        self.controller._set_state(ControlState.IDLE, "撬拨定位未完成")
                    self.log(f"撬拨移动未完成：{error}")
                    messagebox.showwarning("撬拨移动未完成", str(error), parent=self)
                else:
                    self.controller.complete_pry_workflow()
                    self.log("撬拨夹爪已到达夹持点，夹爪保持张开")
                    messagebox.showinfo("撬拨定位完成", "机械臂已到达夹持点，夹爪保持张开。", parent=self)
            self.after(0, finish)
        threading.Thread(target=worker, name="platform-a-pry-move", daemon=True).start()

    def start_clamp(self):
        if self.controller.device.read_only and self.active_branch is None:
            self.controller.begin_clamp_preview()
            self.active_branch = "clamp"
            self.home_button.configure(state="normal")
            self.clamp_execute_button.configure(state="normal")
            self.control_buttons[0].configure(state="disabled")
            self.log("已进入夹挤分支：仅显示原视觉，尚未计算路线或运动")
            return
        self.execute_clamp()

    def execute_clamp(self):
        if not self.controller.device.read_only:
            self.safe(self.controller.start_clamping)
            return
        if self.operation_running:
            messagebox.showwarning("操作正在进行", "请等待当前动作完成。", parent=self)
            return
        clamp_mm = self.param_entries["夹挤位移(mm)"].get().strip()
        if not messagebox.askyesno(
            "确认开始夹挤",
            f"机械臂将根据当前摄像头画面规划路线，移动到足跟并夹挤 {clamp_mm} mm。\n"
            "请确认足模型位置没有改变、运动范围内无人，并可随时按下实体急停。",
            parent=self,
        ):
            return
        if not self.apply_params():
            return
        self.operation_running = True
        self.clamp_button.configure(state="disabled")
        self.home_button.configure(state="disabled")
        self.log("夹挤命令已提交：将先规划完整路线，再移动和夹挤")
        def worker():
            error = None
            try:
                self.controller.start_clamping()
            except Exception as exc:
                error = exc
            def finish():
                self.operation_running = False
                self.clamp_button.configure(state="normal")
                self.home_button.configure(state="normal")
                if error is not None:
                    self.log(f"夹挤未完成：{error}")
                    messagebox.showwarning("夹挤未完成", str(error), parent=self)
                else:
                    messagebox.showinfo("夹挤完成", "机械臂已到达夹持位置，并完成设定的夹挤动作。", parent=self)
            self.after(0, finish)
        threading.Thread(target=worker, name="platform-a-clamp", daemon=True).start()

    def refresh(self):
        try:
            s=self.controller.tick(.1); w=s.wrench; p=s.pose
            for k,val in (("fx",w.fx),("fy",w.fy),("fz",w.fz),("tx",w.tx),("ty",w.ty),("tz",w.tz),("force",w.force_norm),("x",p.x),("y",p.y),("z",p.z),("opening",self.controller.device.gripper_opening)): self.vars[k].set(f"{val:.2f}")
            prefix="真机监测" if self.controller.device.read_only else "离线仿真"
            self.state_var.set(f"{prefix}｜{self.controller.state.value}")
            self.progress_var.set(s.progress); self.draw_scene()
        finally: self.after(100,self.refresh)

    def draw_scene(self):
        if not hasattr(self,"canvas"): return
        c=self.canvas; c.delete("all"); w=max(c.winfo_width(),500); h=max(c.winfo_height(),300)
        if self.controller.device.read_only:
            device = self.controller.device
            mode = getattr(device, "vision_mode", "idle")
            if mode in ("pry", "clamp"):
                # 新算法首帧尚未完成时，仍显示状态服务已收到的原始 D435
                # 画面，避免界面停在“等待 D435”而看不到视频。
                result = device.pry_vision.result
                raw_frame = getattr(device, "camera_frame_png", None)
                overlay_frame = device.pry_vision.frame_png
                # Keep the worker's stable YOLO/diagnostic frame whenever it
                # exists. Switching between raw and overlay frames on a
                # transient invalid result caused flashing and hid markings.
                frame = overlay_frame or raw_frame
            else:
                # Keep the live D435 image visible after reset/stop even when
                # the overlay worker is intentionally no longer running.
                frame = getattr(device, "camera_frame_png", None)
                result = {}
            if frame:
                if frame is not self.camera_frame_bytes:
                    image=tk.PhotoImage(data=base64.b64encode(frame))
                    factor=max(1,int(max(image.width()/w,image.height()/h)+0.999))
                    self.camera_photo=image.subsample(factor,factor) if factor>1 else image
                    self.camera_frame_bytes=frame
                c.create_image(w/2,h/2,image=self.camera_photo,anchor="center")
                image_w=float(result.get("image_width") or 0)
                image_h=float(result.get("image_height") or 0)
                if result.get("heel_detected") and image_w>0 and image_h>0:
                    shown_w=self.camera_photo.width(); shown_h=self.camera_photo.height()
                    x0=(w-shown_w)/2; y0=(h-shown_h)/2
                    def screen(point):
                        return (x0+point[0]*shown_w/image_w,y0+point[1]*shown_h/image_h)
                    a=result.get("clamp_contact_a_px"); b=result.get("clamp_contact_b_px")
                    if a and b:
                        ax,ay=screen(a); bx,by=screen(b)
                        c.create_line(ax,ay,bx,by,fill="#ffd400",width=3)
                        for px,py in ((ax,ay),(bx,by)):
                            c.create_oval(px-5,py-5,px+5,py+5,fill="#ffd400",outline="")
                    center=result.get("heel_center_px")
                    if center:
                        cx,cy=screen(center)
                        c.create_line(cx-8,cy,cx+8,cy,fill="white",width=2)
                        c.create_line(cx,cy-8,cx,cy+8,fill="white",width=2)
            else:
                c.create_text(w/2,h/2,text="等待 D435 真实画面",fill="#6b7680",font=("Microsoft YaHei UI",14))
            return
        c.create_text(15,15,anchor="nw",text="离线仿真视图（非医学影像）",fill="#6b7680")
        c.create_oval(w*.32,h*.35,w*.7,h*.72,fill="#e6c3a5",outline="#9c6d50",width=3)
        c.create_polygon(w*.38,h*.52,w*.48,h*.37,w*.62,h*.49,w*.57,h*.66,w*.42,h*.67,fill="#f7eadc",outline="#8b6d57",width=2)
        z=float(self.vars["z"].get()) if not self.controller.device.read_only else 0.0
        x=float(self.vars["x"].get()) if not self.controller.device.read_only else 0.0
        c.create_line(w*.5,h*.14,w*.5,h*(.43-z/120),fill="#286a9b",width=8,arrow="last")
        gap=max(25,75-x*2); c.create_line(w*.29,h*.55,w*.5-gap,h*.55,fill="#3c4650",width=10); c.create_line(w*.71,h*.55,w*.5+gap,h*.55,fill="#3c4650",width=10)
        c.create_text(w*.5,h*.84,text=f"撬拨 {z:.1f} mm    夹挤 {x:.1f} mm",font=("Microsoft YaHei UI",12,"bold"),fill="#163c5a")

    def log(self,message):
        if not hasattr(self,"log_text"): return
        self.log_text.configure(state="normal"); self.log_text.insert("end",f"[{datetime.now():%H:%M:%S}] {message}\n"); self.log_text.see("end"); self.log_text.configure(state="disabled")

    def export(self):
        try: folder=self.store.export(); self.log(f"记录已导出：{folder}"); messagebox.showinfo("导出完成",f"文件已保存至：\n{folder}",parent=self)
        except Exception as e: messagebox.showerror("导出失败",str(e),parent=self)
