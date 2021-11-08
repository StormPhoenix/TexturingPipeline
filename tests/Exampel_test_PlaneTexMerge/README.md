### Plane Texture Merge

- 只在稀疏网格表面融合纹理。思路：在网格表面检测出平面，将所有相机投影到平面生成纹理图像 Surface_image，
这些图像进行比较检查光照不一致区域，例如行人车辆。        
        
### Test Cases
    
- aym
    - work directory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/aym
    - options: --input_mesh ./Input/model_merge_tex.ply --output_mesh ./Output_PlaneMerge/model.ply --scene_file ./visualSFM/nvm_anyuanmen.nvm --distort_dir ./Output_PlaneMerge/tmp
   
        