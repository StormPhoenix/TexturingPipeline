### Texture Remeshing

- 待解决问题 
    - 部分平面会遗漏
        - MeshPolyRefinement 的 minSize 设置成 1 不行，因为每个 face 会被打个分，分低就会被过滤。
    - 采样引起的抗锯齿（已解决 Gauss blur）
    - 利用 PCA 计算 m_x_axis \ m_y_axis 不太考虑，方向具有随机性
    - 纹理块生成过程中动态选取分辨率（太模糊了；elephant 报纸上的数据）
    - 纹理块的 padding 问题 （已解决）
    - 纹理压缩
        - TextureRemeshing 生成的纹理明显有点问题，分开的间隔太大了。
        
### Test Cases
- Debug
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Output_debug/debug.obj --output_mesh ./Output_debug_remesh/debug.obj
    
- aym
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Output_i23d/model.obj --output_mesh ./Output_i23d_remesh/model.obj
    
- Debug
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Input/debug.ply --output_mesh ./Output_ply_remesh/debug_ply_remesh.obj
    
- elephant
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/elephant
    - options: --input_mesh ./Output_i23d/model_0.obj --output_mesh ./Output_i23d/remeshing/remeshing.obj
        