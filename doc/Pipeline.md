#### Planning
- TextureAtlas
    - 思考如何填充黑色区域
    - 采样分辨率重新设计
    - TextureAltas 是否有分辨率大小限制
    - MeshPolyRefinement 平面大小限制
    
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

#### View Selection
- MRF
- Projection

#### Data Structure
- Model Data Structure

#### Papers
- Keep reading paper per-day

#### Bugs
- const 常量引用 std::vector，当 vector 扩展时，常量值可能会改变。