# overview
 本项目将机械臂的平行四边形结构简化，当作3R机械臂。可以输入θ角获得正解，也可以输入XYZ位置来获得逆解。

# 使用方法
 本项目使用了peter corke的rvc toolbox，安装方式：https://petercorke.com/toolboxes/robotics-toolbox/ 
 解压文件夹后，运行RRR_Robot.m。弹出gui后填入数据，点击forward或者inverse进行结算与画图。若要更改机械臂参数，在两个button的callback函数中进行更改。
