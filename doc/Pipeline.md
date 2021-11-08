#### Planning
- MRF Algorithms
    - 导入 MRF 库（mapmap_cpu \ LBP \ GCoptimization）
        - mapmap_cpu 和 LBP 实际上是同一套算法，目前考虑试用 LBP 代码（先弄清楚用法）
    
- TextureAtlas
    - 思考如何填充黑色区域
        - 黑色区域像素保存该点起始位置处最大矩形大小
        - 单个 face 区域是否应该剔除
        
    - 采样分辨率重新设计、TextureAltas 是否有分辨率大小限制
    
    - 参考 Seamless Texture Atlas（patch 生成问题暂时搁置，先解决 patch 排布问题）
        - texture patch 的排布都是 rectangle 不存在空隙的问题。
        - 生成 quadratic patch 的方式值得参考。
    
    - MeshPolyRefinement 平面大小限制
    
    
- [MakeDense](../tests/Example_test_MakeMeshDense/README.md)
    - 加密网格与未加密网格之间的联系用 color 标记 -> 用未加密网格做 planar detection -> 每个 planar 内部用加密后的网格 uv 信息将纹理复制到新纹理图上
        -> 计算未加密网格在新纹理图上的 uv
        
- [图像融合](../tests/Exampel_test_PlaneTexMerge/README.md) - 多个相机投影在 plane 上进行融合
    <font color="red">(由于不能利用几何信息，遮挡性质难以判断，会加大巨量 labels 数目)</font>
    - 考虑 1：Graph-cuts 融合
        - 大致看一下 graph-cuts 文章看看是如何做图像融合的
        - 测试 GCoptimization 代码
            - graph-cut-optimization
        
    - 考虑 2：相机两两对比，确定光照不一致区域，对不一致区域做大多数投票，选择"大多数"相机融合
        - "大多数" 相机的选取不一定能够确定。在不能判断遮挡的情况下，无法判断某张图片是 ground-truth
        
    - 考虑 3：还是要利用加密后的网格信息。利用划分后的三角形剔除被遮挡相机、outlier 点。
        - 利用 projection 方法在 plane 上做投影，边界处既可以按照原本的方法试用 poisson editing，或者新考虑到的 min-cut
        - 利用 MRF 方法。和原本算法区别：之前是在整个 Model 上做 MRF，这次是在单个 Planer 上做 MRF 优化。<font color="red">一个额外的操作是：这次可以修改 MRF 函数，使纹理边界尽量分布在弱纹理区域。</font>
            - 对整 Mesh 和单个 Plane 分别做 MRF-Optimization 在视觉上区别不大（纹理边界基本没变过）
    
    - 考虑 4：对网格表面生成图像步骤本身很费时。如果考虑在生成图像前提前检查是否要生成图像？比如在 plane 上随机采样点对相机求交线，如果大多数交线被阻挡，则认为该相机不可取。
    
- Learn math tools
- Add bundle adjustment algorithm
    - BA 算法和基于 color 的优化还是有很大的区别
- Read new mvs-texturing code
- Check --outlier_removal option in mvs-texturing
- Read new code [modified mvs-texturing](https://github.com/Xbbei/mvs-texturing/commit/cf942e1add75230872e0adfa49f588661114ef29)
- Integration
    - Model 如果全修改成 Eigen 格式，工作量太大不建议。最好用 mve 替换过来，并开启 projection

#### Problem
- 输入 260w 稠密网格，效果变差了。没有很好的处理超大平面。对于一块单独的 plane，不能直接用一张纹理贴上去了事。
- 出现了一些不该出现的碎块
- 路牌贴错了
- 建模出现的车辆被误投影为地面。
    - Photo-Consistency Check 没办法处理好动态物体的情况

#### Conceptions
- Homography
    - [单应矩阵的推导与理解](https://zhuanlan.zhihu.com/p/138266214)
    - [单应性矩阵的理解及求解3](https://blog.csdn.net/lyhbkz/article/details/82254893)
    
- Essential Matrix and Fundamental Matrix 
    - [本质矩阵与基础矩阵的区别](https://www.zhihu.com/question/27581884)
    - [基础矩阵求解](https://blog.csdn.net/qq_42399848/article/details/89348740)
    - [基础矩阵求解2](https://blog.csdn.net/baidu_38172402/article/details/83502492)

- Optimization
    - [李群李代数](https://zhuanlan.zhihu.com/p/358455662)
    - [Least squares](https://en.wikipedia.org/wiki/Least_squares)
    
- Unrecognized
    - [LU Decomposition](https://zhuanlan.zhihu.com/p/54943042)
    - [SVD (Single Value Decomposition)](https://zhuanlan.zhihu.com/p/29846048)
    - [QR Decomposition](https://zhuanlan.zhihu.com/p/47251888)
    - [Linear least squares](https://en.wikipedia.org/wiki/Linear_least_squares)
    - [Non-linear least squares](https://en.wikipedia.org/wiki/Non-linear_least_squares)
    - [Gauss Newton](https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm)
    - [Closed-form expression](https://en.wikipedia.org/wiki/Closed-form_expression)
    - [Total least squares](https://en.wikipedia.org/wiki/Total_least_squares)
    - [Levenberg–Marquardt algorithm](https://en.wikipedia.org/wiki/Levenberg%E2%80%93Marquardt_algorithm)

#### Optimization Tools
mainly reference from openMVS and colmap
- MRF - LBP Tools
- MRF - Graph-cuts Tools [SeamlessTex](https://github.com/fdp0525/SeamlessTex/)
- Bundle Adjustment Tools
    - [Bundle Adjustment 原理](https://zhuanlan.zhihu.com/p/344766723)

- [Graph-cut RANSAC](https://github.com/danini/graph-cut-ransac) 
- [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus)
    - [Correspondence Problem](https://en.wikipedia.org/wiki/Correspondence_problem)
    - [Fundamental Matrix](https://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)) (自行推导)
    - 找到 RANSAC 在立体匹配中的应用

#### Photon-Consistency Check
- [colmap](https://github.com/colmap/colmap/blob/9f3a75ae9c72188244f2403eb085e51ecf4397a8/src/mvs/patch_match.h)
- [openMVS](https://github.com/cdcseacave/openMVS/search?q=photo+consistency)
    - 代码和 mvs-texturing 是一样的，仅仅只是优化算法 LBP 不一样。

#### View Selection
- MRF
- Projection

#### Data Structure
- Model Data Structure

#### Papers
- Keep reading paper per-day

#### Bugs
- const 常量引用 std::vector，当 vector 扩展时，常量值可能会改变。