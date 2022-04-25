### RedundantTextureRemoval

剔除网格冗余纹理

### Usage

##### 参数

- input_mesh: 输入模型路径，要求模型格式为 *.obj
- output_prefix: 输出纹理剔除结果路径

##### 案例

> // 输出命名为 RemovalModel.obj 的网格模型
> 
> ./testRedundantTextureRemoval --input_mesh=./model_redundant.obj --output_prefix=./RemovalModel
