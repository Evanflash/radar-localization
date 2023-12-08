* 配准算法推导
待求位姿为$\xi=(x, y, \alpha)$
设对应的位姿变换矩阵为：
$$
T=
\left[
\begin{matrix}
cos\alpha & sin\alpha & x \\
-sin\alpha & cos\alpha & y \\
0 & 0 & 1
\end{matrix}
\right] \tag{1}
$$

残差公式为：
$$ f(x, y, \alpha) = argmin\sum S^2\tag{2}$$

将source点云中的一点$p^*(x_0, y_0)$经过位姿变换到target点云之后，点的坐标转换为：
$$
p=Tp^*=
\left[
\begin{matrix}
x^*cos\alpha + y^*sin\alpha + x\\
-x^*sin\alpha + y^*cos\alpha + y
\end{matrix}
\right]=
\left[
\begin{matrix}
x_0\\
y_0
\end{matrix}
\right]
\tag{3}
$$

设target点云中p周围存在两点$p_1(x_1, y_1)$和$p_2(x_2, y_2)$，则这两点与p形成的三角形的三边长分别为：
$$
a = \sqrt{(x_0 - x_1)^2 + (y_0 - y_1)^2}\\
b = \sqrt{(x_0 - x_2)^2 + (y_0 - y_2)^2}\\
c = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2}
\tag{4}
$$
三角形边长的一半可以表示为：
$$
l=\frac{1}{2}(a + b + c)
\tag{5}
$$

则可以通过海伦公式计算出三角形的面积：
$$
S = \sqrt{l(l-a)(l-b)(l-c)}
\tag{6}
$$

定义误差为
$$
e_i=S=\sqrt{l(l-a)(l-b)(l-c)}\\
=\frac{1}{4}\sqrt{-a^4-b^4-c^4+2a^2b^2+2a^2c^2+2b^2c^2}
\tag{7}
$$
令：
$$
g(x, y, \alpha)=-a^4-b^4-c^4+2a^2b^2+2a^2c^2+2b^2c^2
\tag{8}
$$
则:
$$
\frac{\partial e_i}{\partial \xi}=\frac{1}{8}\frac{1}{\sqrt{g(x,y,\alpha)}}\frac{\partial g}{\partial \xi}\\=\frac{1}{8}\frac{1}{\sqrt{g(x,y,\alpha)}}(-4a^3\frac{\partial a}{\partial \xi}-4b^3\frac{\partial b}{\partial \xi}
+4ab^2\frac{\partial a}{\partial \xi}+4a^2b\frac{\partial b}{\partial \xi}+4ac^2\frac{\partial a}{\partial \xi}+4bc^2\frac{\partial b}{\partial \xi})
\tag{9}
$$
其中：
$$
\frac{\partial a}{\partial \xi}=\frac{1}{2}\frac{1}{a}[2(x_0-x_1)\frac{\partial x_0}{\partial \xi} + 2(y_0-y_1)\frac{\partial y_0}{\partial \xi}]
\tag{10}
$$
$$
\frac{\partial b}{\partial \xi}=\frac{1}{2}\frac{1}{b}[2(x_0-x_2)\frac{\partial x_0}{\partial \xi} + 2(y_0-y_2)\frac{\partial y_0}{\partial \xi}]
\tag{11}
$$
$$
\frac{\partial x_0}{\partial \xi}=
\left[
\begin{matrix}
1 & 0 & -x^*sin\alpha + y^*cos\alpha 
\end{matrix}
\right]
\tag{12}
$$
$$
\frac{\partial y_0}{\partial \xi}=
\left[
\begin{matrix}
0 & 1 & -x^*cos\alpha - y^*sin\alpha 
\end{matrix}
\right]
\tag{13}
$$


* 点云去畸变与多普勒补偿
采用恒速模型，假设三帧之间的两个位姿变换矩阵一样，则有：
$$
T_t = T_{t-1}=
\left[
\begin{matrix}
cos\alpha & sin\alpha & x \\
-sin\alpha & cos\alpha & y \\
0 & 0 & 1
\end{matrix}
\right]
\tag{1}
$$

则有角速度与线速度分别为：
$$
w = \frac{\alpha}{\delta t}
\tag{2}
$$
$$
v_x = \frac{x}{\delta t},
v_y = \frac{y}{\delta t}
\tag{3}
$$
设雷达扫描开始时刻的方向为$\theta_0$（雷达扫描角度为雷达扫描方向与车辆行驶方向的夹角），对于极坐标系中某一个数据点$p(r,\theta)$，首先将当前车辆坐标系的x,y轴与前一帧车辆坐标系对齐，则有：
$$
\theta^* = \theta - \frac{\theta - \theta_0}{2\pi}
\tag{4}
$$
再去除其由多普勒效应产生的距离测量误差：
$$
r^*=r+\beta(v_xsin\theta^* + v_ycos\theta^*)
\tag{5}
$$
