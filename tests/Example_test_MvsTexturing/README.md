### MVS Texturing

- 待解决问题
    - 引入 GCOptimizatin
    
- Mapmap-Cpu 与 LBP 对比结果
    - 速度：同样测试用例，Mapmap-Cpu 速度是 LBP 的十倍 <font color="red">?(检查下 mapmap 和 LBP 的截止条件)</font>
    - 效果：
        - LBP 效果比 Mapmap-Cpu 更好。Mapmap 平坦路贴的面片不连续，看起来一块一块的纹理都不一致
        - 使用了 LBP 的效果比 openMVS 要好。贴的纹理是大块大块的，不再有模糊。<font color="red">?(检查下 data term 的计算)</font>
        
- 关于投影方法的思考
    - 本质：投影方法是希望大块三角面片采用同样一张图像去贴。
    ```在本质上，投影方法和 MRF 的区别是引入了人为先验条件。
    1）MRF 认为某一个面片对相机的倾向是依靠 data cost
    来衡量的，Projection 则认为不需要像 data cost 这么严格，如果这个面片能够和周围面片达成一致，则允许 data cost 在一定范围内是等价的；
    2）MRF 认为相邻面片的
    smooth term 的一致性应该靠 Potts 模型来表示，然而投影方法认为，及时相邻面片选择了不同相机，只要相机图像内容是一致的，那也是可以选的。
  
    根据以上分析：其实可以把投影方法转化为 MRF 方法，修改 SmoothCost 为 truncated L2 distance，(第一个想到的 metric 是 mean color)，
    修改 DataCost 为 quality 的百分比。
    ```
  
    - 人为实现投影方法
        - 给定平面让相机投影，原本的做法是考虑相机能够覆盖的面数、每个面上相机倾斜角来决定用那种相机覆盖。
        但现在看来应该考虑使用 projection - info 的信息（只考虑 cos<face_normal, camera_ray> 和 face overlap rate 是不够的，这忽略了图像信息）
        
        - 每次只选一张相机用于 plane，未覆盖到的区域就不管了，这显然不合适。<font color="red">这个容易解决。选取相机往上贴之前需要对每个相机打分，按照得分排行依次覆盖剩余未覆盖平面</font>
        
        - 对于一个平面，每个相机只有两种状态：投射区域要么全贴，要么不贴（不考虑 photo-metric 判断的情况）。这和实际情况不符：存在一个大平面，投射的部分区域由于和相机
        过远导致视角倾斜图像失真。
        
    - 人为投影方法无法消除 "不存在物体" 的边缘
        - "不存在物体" 是通过 photo-metric 消除的，但 photo-metric 无法细化到检测 "不存在物体" 的边缘 
    
### Test Cases
- aym(normal case)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_labeled.ply --output_prefix ./Output_TexturePipeline/Textured_model

- aym(debug case)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_labeled.ply --output_prefix ./Output_TexturePipeline/Textured_model --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_global_seam_leveling true --skip_local_seam_leveling true --skip_hole_filling true