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
- texture_quality : 仅当 sparse_model 设置为 true 时，texture_quality 开始起作用，用于指使输出纹理质量，取值区间为 [0, 1]，取值越大纹理质量越好，运行效率越低；

###### 普适选项建议

- 选项 `outlier_removal` 建议选择 `gauss_clamping`

###### 针对稀疏网格的选项建议

- 对稀疏网格贴纹理，要得到效果较好的纹理，该网格最好带上 face color 属性
- `sparse_model` 需要设置成 true；对应的，如果运行速度较慢，可以尝试减小 `texture_quality` 的值，降低纹理质量，增加运行速度；

## Tips
- `method_type` 设置成 `submrf` 其实效果也不错。

###### Examples
- M1(Projection)
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/M1
    - Options: --scene_file ./visualSFM/M1.nvm --input_mesh ./input/model_0.ply --output_prefix ./Output_TexturePipeline/model_proj --mrf_call_lib mapmap --outlier_removal gauss_clamping --method_type projection --keep_unseen_faces true --skip_hole_filling true

- elephant
    - WorkDirectory: /Users/stormphoenix/Workspace/Projects/CLionProjects/3dReconstruction/Dataset/elephant
    - Projection|无Outlier: --scene_file ./visualSFM/test.nvm --input_mesh ./Input/model_0_labeled.ply --output_prefix ./Output_TexturePipeline/model_Projection --mrf_call_lib mapmap --method_type projection --keep_unseen_faces true --skip_hole_filling true --sparse_model=false --debug_mode=true
    - Mapmap|无Outlier: --scene_file ./visualSFM/test.nvm --input_mesh ./Input/model_0_labeled.ply --output_prefix ./Output_TexturePipeline/model_Mapmap_on-outlier --mrf_call_lib mapmap --method_type mrf --keep_unseen_faces true --skip_hole_filling true --sparse_model=false --debug_mode=true
