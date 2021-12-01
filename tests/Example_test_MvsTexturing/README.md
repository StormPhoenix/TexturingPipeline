### MVS Texturing

##### Test example
```
   ./testMvsTexturing --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/vis2mesh_sim.ply --output_prefix ./Output_TexturePipeline/vis2mesh_sim_MakeDense --mrf_call_lib openmvs --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true --sparse_model true --debug_mode true
```

##### Program options

###### 必选

- scene_file : 场景文件，要求 *.nvm 格式
- input_mesh : 输入模型，要求 *.ply 格式。<font color="red">（如果输入模型是稀疏网格，则该网格必须带 face color 属性才能保证输出较好效果）</font>
- output_prefix : 输出结果命名的前缀，不带任何后缀名。eg ` ./output/model`

###### 可选
- method_type : 纹理映射算法，仅支持两类 `{mrf}, {projection}`，默认采用 `{mrf}`
- mrf_call_lib : MRF 算法的运行库，仅当 `method_type = mrf` 时起作用。目前支持两类 `{mapmap}, {openmvs}`。`openmvs` 的效果更好，但运行时间长，`mapmap` 效果会差一点，但速度快。
- outlier_removal : 纹理表面异物剔除选项，支持三种剔除方式 `{none}, {gauss_clamping}, {gauss_damping}`，默认使用不剔除异物选项 `none`。<font color="red">（建议采用 gauss_clamping )</font>
- keep_unseen_faces : 若存在面片不被任何相机覆盖，可选择是否保留该面片。可选项有 `{true}, {false}`
- sparse_model : 如果输入模型是稀疏网格，则必须设置为 `true` 才能保持较好效果。可选项有 `{true}, {false}`
- debug_model : 用于调试时输出中间文件，一般不设置。可选项有 `{true}, {false}`

###### 普适选项建议

- 选项 `outlier_removal` 建议选择 `gauss_clamping`

###### 针对稀疏网格的选项建议

- 对稀疏网格贴纹理，要得到效果较好的纹理，该网格最好带上 face color 属性。`sparse_model` 需要设置成 true


### Problems

- 待解决问题
    - 稀疏网格纹理重映射问题。网格重映射算法在 TextureRemeshing 已经被实现。但对于稀疏网格，对未被检测到平面的网格区域做重映射，
    会存在 one triangle (sparse) 跨 multiple triangle (dense) 的问题。这样就没办法把纹理合并起来。解决办法是：如果某部分网格没有被检测
    为平面，就不对这块区域的 triangle 做划分。可是这个办法太依赖输入网格的平面检测结果了。万一这个区域本身很稀疏却没有被检测为平面，一旦不划分，
    可见性检测就会出问题。
    那应该这样解决，按照原有方式做网格划分，如果一个三角形被划分了，但是却没被检测为平面，我还是认为这个三角形是一个平面，重映射时就用这个拟合平面
    来重映射。

    - 引入 GCOptimizatin
    - 用 elephant 做测试数据，投影结果非常怪异（发现检测得到的平面基本上弯曲度都比较大）
    
    - 用于运行平面检测的程序设置了新参数，最终跑出的模型面片之间大小差异非常大，纹理映射效果变得非常差了。
        - 应对策略：密集部位采用求解 MRF 方式；稀疏部位用 Projection 方法;
        
    - 用新参数设置 elephant 模型，得到了非常多的细碎无纹理三角片。原因是模型投影到自定义平面，由于 plane_density 是人为设置的值，所以投影到平面上可能还不到1像素
        - plane_density 的大小要动态确定
        - 没有纳入 plane detection 的面片也要尝试组合成一个 patch
        - 如果真的遇到极端情况比如一个 face 的结果投影到平面都不止 1 像素，该考虑如何解决。
    
- 稀疏网格纹理映射
    - MvsTexturing、MakeMeshDense 和 TextureRemeshing 已经写好，稀疏网格纹理映射就靠这三个库串起来即可。
    - TextureRemeshing 还存在一些小问题，就是对纹理采样得到的图像变得模糊、有锯齿，还没想出改进办法。
    
- Mapmap-Cpu 与 LBP 对比结果
    - 速度：同样测试用例，Mapmap-Cpu 速度是 LBP 的十倍 <font color="red">?(检查下 mapmap 和 LBP 的截止条件)</font>
    - 效果：
        - LBP 效果比 Mapmap-Cpu 更好。Mapmap 平坦路贴的面片不连续，看起来一块一块的纹理都不一致
        - 使用了 LBP 的效果比 openMVS 要好。贴的纹理是大块大块的，不再有模糊。<font color="red">?(检查下 data term 的计算，查出原因很有必要，效果提升需要一个解释 !!!)</font>
        
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
        
        - 每次只选一张相机用于 plane，未覆盖到的区域就不管了，这显然不合适。<font color="red">这个容易解决。选取相机往上贴之前需要对每个相机打分，按照得分排行依次覆盖剩余未覆盖平面（已解决）</font>
        
        - 对于一个平面，每个相机只有两种状态：投射区域要么全贴，要么不贴（不考虑 photo-metric 判断的情况）。这和实际情况不符：存在一个大平面，投射的部分区域由于和相机
        过远导致视角倾斜图像失真。
        
    - 人为投影方法无法消除 "不存在物体" 的边缘
        - "不存在物体" 是通过 photo-metric 消除的，但 photo-metric 无法细化到检测 "不存在物体" 的边缘
            - 通过比较 face - image 的 gauss value 方式，可以消除一部分边缘错拼的问题，但依然有一些位置无法消除。我的解释是：photo-metric 指标
            是 face - image 的 mean-color，采用均值作为特征过滤掉了图像上很多梯度（方差）信息。<font>（如何在特征值方面入手？）</font>
            - 无脑对冲突区域进行扩张。在扩张同时还会检查 face visibility 和 photo metric 来剔除 outlier <font>（这个办法效果还是很好的。但扩张又导致了一些没必要扩张的区域也扩张了，所以后续还要考虑对扩张的"种子"位置做筛选）</font>
    
### Test Cases
- aym(normal case)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_labeled.ply --output_prefix ./Output_TexturePipeline/Textured_model

- aym(dense case + openmvslib)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_dense_vis2mesh.ply --output_prefix ./Output_TexturePipeline/model_dense_openmvs --mrf_call_lib openmvs --outlier_removal gauss_clamping --method_type mrf --keep_unseen_faces true --skip_hole_filling true

- aym(dense case + mapmaplib)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_dense_vis2mesh.ply --output_prefix ./Output_TexturePipeline/model_dense_mapmap --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type mrf --keep_unseen_faces true --skip_hole_filling true

- aym(sparse case + openmvslib)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/vis2mesh_sim.ply --output_prefix ./Output_TexturePipeline/model_sparse_openmvs --mrf_call_lib openmvs --outlier_removal gauss_clamping --method_type mrf --keep_unseen_faces true --skip_hole_filling true --sparse_model=true --debug_mode=true


- aym(debug case)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_labeled.ply --output_prefix ./Output_TexturePipeline/Textured_model --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_global_seam_leveling true --skip_local_seam_leveling true --skip_hole_filling true
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/model_labeled_sim_2.ply --output_prefix ./Output_TexturePipeline/Textured_model --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true
    - Options: --scene_file ./visualSFM/nvm_anyuanmen.nvm --input_mesh ./Input/debug.ply --output_prefix ./Output_TexturePipeline/debug_make_dense --mrf_call_lib openmvs --outlier_removal gauss_clamping --method_type mrf --keep_unseen_faces true --skip_hole_filling true
    
- M1
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/M1
    - Options: --scene_file ./visualSFM/M1.nvm --input_mesh ./input/model_0.ply --output_prefix ./Output_TexturePipeline/model_proj --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true
    
- elephant
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/elephant
    - Options: --scene_file ./visualSFM/test.nvm --input_mesh ./Input/model_0.ply --output_prefix ./Output_TexturePipeline/model_proj --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true
    
- elephant01    
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/elephant01
    - Projection Options: --scene_file ./visualSFM/model.nvm --input_mesh ./model_dense_vis2mesh_simplified.ply --output_prefix ./Output_TexturePipeline/model --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true --sparse_model true
    - OpenMVS Options: --scene_file ./visualSFM/model.nvm --input_mesh ./model_dense_vis2mesh_simplified.ply --output_prefix ./Output_TexturePipeline/model --mrf_call_lib openmvs --outlier_removal gauss_clamping --method_type mrf -- --keep_unseen_faces true --skip_hole_filling true --sparse_model true
            