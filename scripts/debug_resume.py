from fairino import Robot

r = Robot.RPC("192.168.58.2")
print("rpc", r is not None)
print("mode", r.Mode(0))
print("enable", r.RobotEnable(1))
print("resume", r.ResumeMotion())
print("program_resume", r.ProgramResume())
r.CloseRPC()
