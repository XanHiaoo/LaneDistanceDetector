<!DOCTYPE html>
<html>
<head>
<style>
  .container {
    display: flex;
    align-items: flex-start; /* 修改为垂直对齐顶部 */
  }

.left {
flex: 1;
padding: 20px;
border: 1px solid #ccc;
}

.right {
flex: 1;
text-align: center;
padding: 20px;
border: 1px solid #ccc;
}

img {
max-width: 100%;
height: auto;
}
</style>
</head>
<body>
<div class="container">
  <div class="left">
    <h2>算法流程简述</h2>
    <h3>过滤直线 (FilterLines)</h3>
    <p>接受线段的向量作为输入 计算每条线段的斜率（m）和截距（c）</p>
    <p>根据线段的斜率和角度进行筛选（拒绝接近水平和垂直的线段）</p>
    <p>在向量中存储相关的线段信息，包括端点、斜率、截距和长度 可选地，将线段数量限制为前15条最长的线段</p>
    <h3>检测直线 (GetLines)</h3>
    <p>应用霍夫线变换来检测边缘图像中的线段</p>
    <p>将线段的坐标转换回原始图像坐标空间</p>
    <p>调用FilterLines函数以筛选和选择相关的线段</p>
    <h3>计算消失点 (GetVanishingPoint)</h3>
    <p>接受一个经过筛选的线段的向量作为输入</p>
    <p>实现类似RANSAC的算法来寻找消失点</p>
    <p>遍历两条线段的组合，计算它们的交点并评估误差 选择具有最小误差的消失点</p>
    <h3>计算消失点 (calculateVanishingPoint)</h3>
    <p>调用GetLines()来检测图像中的线条</p>
    <p>调用GetVanishingPoint()来计算消失点</p>
  </div>
  <div class="right">
    <img src="https://imgurl-x.oss-cn-hangzhou.aliyuncs.com/xuxing-img/image-20230905171152369.png" alt="算法流程图">
  </div>
</div>
<div class="container">
  <div class="left">
    <h3>计算平行平面 (calculatingParallelPlane)</h3>
    <p>计算通过消失点的线的方向向量</p>
    <p>计算由点对形成的平面的交线</p>
    <p>计算两条线共面平面的系数 将结果存储在planeParameters_中</p>
    <h3>计算平面参数 (calculatePlaneParameters)</h3>
    <p>使用先前计算的verticalLinePair_之间的距离和planeParameters_计算平面方程的系数，并更新planeParameters_</p>
  </div>
  <div class="right">
    <h3>计算点之间的距离 (calculateDistanceBetweenPoints)</h3>
    <p>计算两点的方向向量 使用planeParameters_计算平面上的交点 计算交点之间的距离并返回</p>
    <h3>加载设置 (loadSettings)</h3>
    <p>从 JSON 文件加载设置 根据结果设置settingsLoaded并初始化必要的参数 如果设置待处理，保存平面参数</p>
    <h3>保存平面参数 (savePlaneParameters)</h3>
    <p>如果设置是LOAD_PENDING，将平面参数保存到设置中</p>
  </div>
</div>
</body>
</html>
