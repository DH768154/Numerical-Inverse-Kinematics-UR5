# Numerical-Inverse-Kinematics-UR5
 Numerical Inverse Kinematics for UR5 (resolved rate control)



这个文件是写给我老同学的，为了简单并详细地解释Jacobian和运动学逆解地方法，程序中并没有使用循环，而是矩阵一个一个拼起来。

为了直观看出运动学逆解地原理，我将step size，或者叫learning rate设定得很小，这样可以看出机器人从起始点到终点的路径，这也被叫做resolved rate control。

---

Main Function: <u>ur5_rrcontrol.m</u>

<u>@SimpleRobotRJ</u> is for simulation to see the animation only, nothing about inverse kinematics.
