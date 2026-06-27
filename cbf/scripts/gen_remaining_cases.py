#!/Users/qingxu/.ai-env/bin/python3
"""一次性生成补漏的测试用例 JSON 文件"""
import json, os

base = os.path.join(os.path.dirname(__file__), "..", "tests", "cppTest", "unit")

data = {
  "computeObstacleAffineConstraint": {
    "function_name": "computeObstacleAffineConstraint",
    "description": "HOCBF 仿射约束生成测试，对照解析公式手算结果",
    "test_cases": [
      {"id":"TC001_static_far","egoVelocity":10.0,"wheelBase":2.7,"alpha1":1.0,"alpha2":1.0,
       "dxEgo":20.0,"dyEgo":0.0,"vRxEgo":-10.0,"vRyEgo":0.0,
       "axObEgo":0.0,"ayObEgo":0.0,"safetyRadius":4.0,
       "betaAExp":-40.0,"betaTExp":0.0,"boundExp":-(200.0+2.0*(-400.0)+1.0*(400.0-16.0)),"tolerance":1e-6},
      {"id":"TC002_zero_velocity","egoVelocity":0.0,"wheelBase":2.7,"alpha1":1.0,"alpha2":1.0,
       "dxEgo":5.0,"dyEgo":3.0,"vRxEgo":-1.0,"vRyEgo":0.0,
       "axObEgo":0.0,"ayObEgo":0.0,"safetyRadius":4.0,
       "betaAExp":-10.0,"betaTExp":0.0,
       "boundExp":-(2.0*1.0+2.0*(-10.0)+1.0*(5.0*5.0+3.0*3.0-16.0)),"tolerance":1e-6}
    ]
  },
  "clampControlInput": {
    "function_name": "clampControlInput",
    "description": "控制输入双通道限幅测试",
    "test_cases":[
      {"id":"TC001_in_range","aRaw":1.0,"dfRaw":0.1,"aMin":-7.0,"aMax":7.0,"dfMin":-0.5,"dfMax":0.5,"aExp":1.0,"dfExp":0.1,"tolerance":1e-12},
      {"id":"TC002_above_max","aRaw":12.0,"dfRaw":1.0,"aMin":-7.0,"aMax":7.0,"dfMin":-0.5,"dfMax":0.5,"aExp":7.0,"dfExp":0.5,"tolerance":1e-12},
      {"id":"TC003_below_min","aRaw":-10.0,"dfRaw":-1.0,"aMin":-7.0,"aMax":7.0,"dfMin":-0.5,"dfMax":0.5,"aExp":-7.0,"dfExp":-0.5,"tolerance":1e-12}
    ]
  },
  "solveQpHildreth": {
    "function_name": "solveQpHildreth",
    "description": "Hildreth QP 求解器测试，覆盖无约束/上界/下界三种典型场景",
    "test_cases":[
      {"id":"TC1_unconstrained","desc":"无约束最优解 (-Qd^-1 c)","expected_u":[2.0,1.0]},
      {"id":"TC2_clip_top","desc":"约束 a<=1 → a=1, t=1","expected_u":[1.0,1.0]},
      {"id":"TC3_clip_bottom","desc":"约束 a>=3 → a=3, t=1","expected_u":[3.0,1.0]}
    ]
  },
  "cbfControlRevision": {
    "function_name": "cbfControlRevision",
    "description": "CBF 顶层入口测试",
    "test_cases":[
      {"id":"TC1_far_obstacle","desc":"远处障碍物，输出近似原始控制"},
      {"id":"TC2_imminent_collision","desc":"近距离正前方接近，输出加速度应明显小于 0"},
      {"id":"TC3_no_obstacle","desc":"无障碍物 → 输出=原始指令"}
    ]
  }
}
for k,v in data.items():
    p = os.path.join(base, k)
    os.makedirs(p, exist_ok=True)
    with open(os.path.join(p, f"{k}_cases.json"),"w",encoding="utf-8") as f:
        json.dump(v,f,ensure_ascii=False,indent=2)
print("OK")
