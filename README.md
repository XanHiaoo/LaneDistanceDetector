# LaneDistanceDetector单目车道测距

### V1.0,September , 2023

**Authors:** xiao

## 简述

通过一组与车道线垂直的点对和车道线的消失点，计算图像像素与真实世界的坐标转换关系，构建路面的三维平面方程，以达成单目车道测距算法的实现




![车轮检测模型动画4](https://imgurl-x.oss-cn-hangzhou.aliyuncs.com/xuxing-img/%E8%BD%A6%E8%BD%AE%E6%A3%80%E6%B5%8B%E6%A8%A1%E5%9E%8B%E5%8A%A8%E7%94%BB4.gif)

![Snipaste_2023-09-08_17-49-37](https://imgurl-x.oss-cn-hangzhou.aliyuncs.com/xuxing-img/Snipaste_2023-09-08_17-49-37.png)

### 应用

斜视角车轮组数量检测

## 算法原理

[算法原理](./doc/算法原理.md)

## 算法流程

[算法流程简述](./doc/算法流程简述html.md)

![image-20230905153757010](https://imgurl-x.oss-cn-hangzhou.aliyuncs.com/xuxing-img/image-20230905153757010.png)

## 输入

### 建模

- **如已知路面对应三维平面方程**
  $$
  Ax+By+Cz+D=0
  $$
  则需要平面参数
  $$
  A,B,C,D
  $$
  **e.g.**

  ```json
  "PlaneParameters": {
          "A": -0.2794382748644901,
          "B": -0.9532282309456822,
          "C": 0.11519631187205111,
          "D": 5.15989900569963
      },
  ```

- **如需要求得路面对应三维平面方程**

  则需要参数摄像机的内参矩阵CameraMatrix和一组在路沿上与车道线垂直的点对VerticalLine
  $$
  CameraMatrix=\begin{bmatrix}
    f_x&0&u_0\\
    0& f_y&v_0\\
    0&0&1
  \end{bmatrix}   
  \quad\quad\quad\quad\quad\quad\quad\quad\quad\quad\quad
  VerticalLine\begin{cases}
   point1\left (x_1,y_1 \right ) \\point2\left ( x_2,y_2 \right ) 
  \end{cases}
  $$
  **e.g.**

  ```json
  "CameraMatrix": [[1471.0,0.0,958.0],
                   [ 0.0,1475.0,715.0],
          		 [0.0,0.0, 1.0]],
  "TemplateVerticalLine": {
          "Lane1": {
              "Point1": [2050,1408],"Point2": [3157,1676]
          }
      }
  ```

  

### 检测

车道上需要检测的点对应的像素坐标

```c++
cv::Point& Pixel
```

## 输出

### 建模

路面对应三维平面方程
$$
Ax+By+Cz+D=0
$$

### 检测

```c++
cv::Vec3d Coordinate
```

