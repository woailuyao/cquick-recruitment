# C0 三位置误差：现有证据边界与待核验数据

原工程中已有 P1/P2/P3 现场测量记录，但不能仅凭 `residual`、历史 58 cm 外参或帧有效率计算独立米制误差。根据考核细则，须使用**不进入算法输入**的参考 X/Y/Yaw，与每点对应的自动定位输出对齐，再计算：

`error_xy = sqrt((estimate_x-reference_x)^2+(estimate_y-reference_y)^2)`

`error_yaw = abs(((estimate_yaw-reference_yaw+180)%360)-180)`（角度单位 deg）

不得凭空填入参考值。需要从已有现场测量/录屏核对三点参考的坐标轴、半场方向、地图版本、yaw 零方向；尤其不能把人工初始化+PnP 与全自动定位混成同一方法的误差。若原记录不足，保持 `UNVERIFIED`。**本文件不是三点验收通过证明。**
