import numpy as np

# 假设你的文件名是 all_poses.npy
arr = np.load('handeye_img_pose/img/pose.npy')  
print("类型：", type(arr))
print("形状：", arr.shape)
print("数据类型：", arr.dtype)
print("内容预览：\n", arr)
