这个程序主要目的是对比分析通过本质矩阵E和基础矩阵F两种方式,求解rt.主要针对8点法和5点法进行对比分析.  
首先通过ORB特征提取算法提取角点,通过匹配进行前后两帧的特征点匹配,再进行矩阵的EF求解.  
  
.
├── CMakeLists.txt  
├── dtl_image_rt.txt  //结果输出  
├── duantouImgUndistortsuo /图片存储的位置  
├── duantouImgUndistortsuo.txt //输入图片名  
├── pose_estimation_2d2d.cpp //主程序,参考视觉质量14讲CH7.3 2D_2D  
├── readme  
└── traj_anyly.m //分析输出结果  
  
  
https://blog.csdn.net/kokerf/article/details/72911561  
https://blog.csdn.net/KYJL888/article/details/104491732  
