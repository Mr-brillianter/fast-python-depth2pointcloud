// 可以参照：https://github.com/gaoxiang12/slambook/blob/master/ch13/dense_RGBD/pointcloud_mapping.cpp
// 使用PCL库，新建彩色点云xyzrgb，然后进行滤波和降采样等|python使用open3d亲测比较好
// 新建一个点云
    PointCloud::Ptr pointCloud( new PointCloud ); 
    for ( int i=0; i<5; i++ )
    {
        PointCloud::Ptr current( new PointCloud );
        cout<<"转换图像中: "<<i+1<<endl; 
        cv::Mat color = colorImgs[i]; 
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        for ( int v=0; v<color.rows; v++ )
            for ( int u=0; u<color.cols; u++ )
            {
                unsigned int d = depth.ptr<unsigned short> ( v )[u]; // 深度值
                if ( d==0 ) continue; // 为0表示没有测量到
                if ( d >= 7000 ) continue; // 深度太大时不稳定，去掉
                Eigen::Vector3d point; 
                point[2] = double(d)/depthScale; 
                point[0] = (u-cx)*point[2]/fx;
                point[1] = (v-cy)*point[2]/fy; 
                Eigen::Vector3d pointWorld = T*point;
                
                PointT p ;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
                p.b = color.data[ v*color.step+u*color.channels() ];
                p.g = color.data[ v*color.step+u*color.channels()+1 ];
                p.r = color.data[ v*color.step+u*color.channels()+2 ];
                current->points.push_back( p );
            }
        // depth filter and statistical removal 
        PointCloud::Ptr tmp ( new PointCloud );
        pcl::StatisticalOutlierRemoval<PointT> statistical_filter;
        statistical_filter.setMeanK(50);
        statistical_filter.setStddevMulThresh(1.0);
        statistical_filter.setInputCloud(current);
        statistical_filter.filter( *tmp );
        (*pointCloud) += *tmp;
    }
    
    pointCloud->is_dense = false;
    cout<<"点云共有"<<pointCloud->size()<<"个点."<<endl;



// 这段代码是使用OpenCV和PCL库进行3D点云重建的典型示例。为了加速这段代码，我们可以尝试以下方法：

// 使用并行计算：将循环结构改为并行计算，以充分利用多核处理器的计算能力。在PCL中，可以使用tbb::parallel_for或其他并行计算工具来实现这一点。
// #include <tbb/parallel_for.h>

// void processDepthImage(const cv::Mat& depth, const cv::Mat& color, const Eigen::Matrix4d& T, float depthScale, pcl::PointCloud<pcl::PointXYZRGB>::Ptr current)
// {
//     const int height = depth.rows;
//     const int width = depth.cols;

//     tbb::parallel_for(tbb::blocked_range<int>(0, height),
//                       [&](const tbb::blocked_range<int>& range) {
//                           for (int v = range.begin(); v < range.end(); ++v)
//                           {
//                               for (int u = 0; u < width; ++u)
//                               {
//                                   // ... (your code here)
//                               }
//                           }
//                       });
// }
// 使用向量运算：在计算3D点时，可以使用向量运算来提高计算效率。例如，可以使用Eigen库中的向量运算来简化计算过程。
//  复制
//  插入
//  更多
// Eigen::Vector3d point = (T * Eigen::Vector3d(u, v, 1)).head(3);
// 减少函数调用：在代码中，有一些函数调用的开销，如depth.ptr<unsigned short> ( v )[u]和color.data[ v*color.step+u*color.channels() ]。可以考虑将这些操作移到循环外部，减少函数调用的次数。

// 使用缓存：在处理图像数据时，可以使用缓存技术来减少内存访问次数，提高计算效率。例如，可以使用PCL的pcl::Buffer类来缓存深度值和颜色值。

// 优化代码：仔细检查代码，确保没有不必要的计算和操作。例如，可以删除不必要的if语句，简化逻辑结构等。

// 总之，要加速这段代码，需要从多个方面进行优化，包括并行计算、向量运算、减少函数调用、使用缓存以及优化代码结构等。