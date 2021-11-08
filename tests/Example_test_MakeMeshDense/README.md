### MakeMeshDense

- 加密网格，保留加密操作前后网格之间联系（未加密为 A；加密后为 B）

- 对 B 做纹理映射，生成 texture patch，并保留 face tex_coord texture_patch 之间的对应关系

- 在 A 上做平面检测，生成 planar patch。根据 A 和 B 之间的关系，将加密后划分出的网格并入 planar patch

- RemeshUtils 是对带纹理 obj 做简化，因此知道 face 对应用的哪张图像；MakeMeshDense 需要知道 face 对应
    的 texture patch 以及对应的 uv 坐标