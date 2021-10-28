### Texture Remeshing

- 待解决问题 
    - 部分平面会遗漏（MeshPolyRefinement 生成的 plane 有 minSize，估计 minSize 设置成 1 就可以了）
    - 采样引起的抗锯齿（Gauss blur）
    - 纹理块生成过程中动态选取分辨率（太模糊了；elephant 报纸上的数据）
    - 纹理块的 padding 问题 （已解决）
        
### Test Cases
- Debug
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Output_debug/debug.obj --output_mesh ./Output_debug_remesh/debug.obj
    
- aym
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Output_i23d/model.obj --output_mesh ./Output_i23d_remesh/model.obj