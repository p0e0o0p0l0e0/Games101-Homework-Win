作业6：BVH相关和SAH查找均已完成。
提交结果图/images/binary.ppm。
当前代码为32个Buckets的SAH查找。

代码修改：
BVH.cpp:
1 BVHAccel(), 添加splitMethod判断，如果为NAIVE则调用原有函数recursiveBuild()，如果为SAH调用recursiveSAHBuild()
2 修改getIntersection名称为getBVHIntersection，与Object的相交函数名区分，并获得交点的判断逻辑，先与bvh整体判断交点，如果通过后则判断是否有左右节点，递归计算交点。
3 添加CalculateCost(), recursiveSAHBuild(), 用于SAH查找。


结论：
按数量平分查找：
构建时间为0s，渲染时间为2s。

SAH查找，buckets数量
为8时：
构建时间1s。渲染时间为3s。
为32时：
构建时间为1s。渲染时间为4s。

并没有得到使用SAH查找效率更高的结论。

如果在计算最小cost时只在第一次选定一个最长轴，而不是x、y、z轴都计算后得到最小cost，buckets数量
为8时：
构建时间为0s。渲染时间为2s。
为32时：
构建时间为0s。渲染时间为2s。

看起来如果只在第一次选定一个最长轴进行最小cost计算效率更高？（代码在BVH.cpp L:148）