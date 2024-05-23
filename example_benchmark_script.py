import numpy as np
import pandas as pd
from some_matterport3d_library import Matterport3DEnvironment
from some_navigation_library import NoMAD, ViNT, GNM, DeepExplorer, PlaceNav, SLING, iPlanner
 
# 初始化测试环境
env = Matterport3DEnvironment()
navigation_modules = {
    "NoMAD": NoMAD(),
    "ViNT": ViNT(),
    "GNM": GNM(),
    "DeepExplorer": DeepExplorer(),
    "PlaceNav": PlaceNav(),
    "SLING": SLING(),
    "iPlanner": iPlanner()
}
 
def evaluate_agent(agent, env):
    success_count = 0
    total_tests = 100  # 设置测试次数
    for _ in range(total_tests):
        env.reset()
        state = env.get_initial_state()
        done = False
        while not done:
            rgbd_data = env.get_rgbd_data()
            pose = env.get_pose()
            topo_map = env.get_topological_map()
            action = agent.act(rgbd_data, pose, topo_map)
            state, reward, done, info = env.step(action)
            if done and info.get('success', False):
                success_count += 1
    success_rate = success_count / total_tests
    return success_rate
 
results = []
 
# 依次测试每个导航模块
for module_name, module in navigation_modules.items():
    print(f"Evaluating {module_name}...")
    success_rate = evaluate_agent(module, env)
    results.append({"Module": module_name, "Success Rate": success_rate})
 
# 将结果转换为DataFrame并保存为CSV文件
results_df = pd.DataFrame(results)
results_df.to_csv("navigation_module_evaluation_results.csv", index=False)
 
print("Evaluation complete. Results saved to navigation_module_evaluation_results.csv.")