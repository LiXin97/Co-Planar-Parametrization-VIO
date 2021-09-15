//
// Created by xin on 7/9/21.
//


#include <ros/ros.h>
#include "gui.hpp"

Gui::Gui()
{
    auto proc_func = [&]{run();};
//    view_thread.reset( new std::thread(proc_func) );
    show_id = 0;
    pause_status = false;
}

void Gui::run()
{
    pangolin::CreateWindowAndBind("VINS",1024,768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::View& d_video = pangolin::Display("imgVideo")
            .SetAspect(752/(float)480);

    pangolin::CreateDisplay()
            .SetBounds(0.0, 0.3, pangolin::Attach::Pix(180), 1.0)
            .SetLayout(pangolin::LayoutEqual)
            .AddDisplay(d_video);


    // 新建按钮和选择框，第一个参数为按钮的名字，第二个为默认状态，第三个为是否有选择框
    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<int> menuShowframe("menu.Show Frame", 0, 0, 1500);

//    pangolin::Var<std::function<void(void)>> next_step_btn("ui.next_step", &next_step);
//    pangolin::Var<std::function<void(void)>> prev_step_btn("ui.prev_step", &prev_step);

    pangolin::Var<bool> next_step_btn("menu.next_step", false, false);
    pangolin::Var<bool> prev_step_btn("menu.prev_step", false, false);

    pangolin::Var<bool> continue_btn("menu.continue", true, true);

    pangolin::Var<bool> menuWhite("menu.Show White",false,true);
    pangolin::Var<bool> menuShowimg("menu.Show img",true,true);
    pangolin::Var<bool> menuShowCamera("menu.Show Camera",true,true);
    pangolin::Var<bool> menuShowPoint("menu.Show Point",true,true);
    pangolin::Var<bool> menuShowMesh("menu.Show Mesh",true,true);
    pangolin::Var<bool> menuShowSingleMesh("menu.Show single Mesh",true,true);


    // Define Camera Render Object (for view / scene browsing)
    // 定义相机投影模型：ProjectionMatrix(w, h, fu, fv, u0, v0, zNear, zFar)
    // 定义观测方位向量：观测点位置：(mViewpointX mViewpointY mViewpointZ)
    //                观测目标位置：(0, 0, 0)
    //                观测的方位向量：(0.0,-1.0, 0.0)
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
            pangolin::ModelViewLookAt(-1.4, -1.7, 2.3, 0., 0., 0., pangolin::AxisNegY)
//            pangolin::ModelViewLookAt(-0.7,0,1.8, 0,0,0,0.0,1.0, 0.0)
    );

//    camera = pangolin::OpenGlRenderState(
//            pangolin::ProjectionMatrix(640, 480, 400, 400, 320, 240, 0.001, 10000),
//            pangolin::ModelViewLookAt(-3.4, -3.7, -8.3, 2.1, 0.6, 0.2,
//                                      pangolin::AxisNegY))

    // Add named OpenGL viewport to window and provide 3D Handler
    // 定义显示面板大小，orbslam中有左右两个面板，昨天显示一些按钮，右边显示图形
    // 前两个参数（0.0, 1.0）表明宽度和面板纵向宽度和窗口大小相同
    // 中间两个参数（pangolin::Attach::Pix(175), 1.0）表明右边所有部分用于显示图形
    // 最后一个参数（-1024.0f/768.0f）为显示长宽比
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));
    d_cam.Activate(s_cam);



    pangolin::GlTexture imageTexture(752, 480, GL_RGB,false,0,GL_BGR,GL_UNSIGNED_BYTE);


    std::vector< std::vector<Eigen::Vector3d> > tris_3d_single_old;
    std::vector< std::vector< Eigen::Vector2d >> tris_pixels_single_old;
    cv::Mat single_img_old;


    while( !pangolin::ShouldQuit() )
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if(menuWhite)
            glClearColor(1.0f,1.0f,1.0f,1.0f);
        else
            glClearColor(0.0f,0.0f,0.0f,0.0f);

        d_cam.Activate(s_cam);
        const double lens = 0.5;
        glLineWidth(2.0);
        glBegin(GL_LINES);
        glColor3f(1.0f,0.0f,0.0f);
        glVertex3f(0,0,0);
        glVertex3f(lens,0,0);


        glColor3f(0.0f,1.0f,0.0f);
        glVertex3f(0,0,0);
        glVertex3f(0,lens,0);

        glColor3f(0.0f,0.0f,1.0f);
        glVertex3f(0,0,0);
        glVertex3f(0,0,lens);

        glEnd();

        if( menuShowframe.GuiChanged() )
        {
            show_id = menuShowframe;
        }

        if(continue_btn)
        {
            show_id = img_dataset.size() - 1;
            menuShowframe = show_id;
            menuShowframe.Meta().range[1] = points_dataset.size() - 1;

            pause_status = false;
        }
        else
        {
            menuShowframe.Meta().gui_changed = true;
            pause_status = true;
//            menuShowframe.GuiChanged()
        }

        if(next_step_btn)
        {
            if( show_id < img_dataset.size() -1 )
            {
                show_id ++;
            }
            else
                show_id = img_dataset.size() - 1;
            next_step_btn = false;

            menuShowframe = show_id;
        }

        if( prev_step_btn )
        {
            if( show_id > 1 )
                show_id --;
            else
                show_id = 0;
            prev_step_btn = false;

            menuShowframe = show_id;
        }

//        menuShowframe = show_id;

        cv::Mat _img;
        std::vector< pangolin::OpenGlMatrix > Twcs_cur;
        std::unordered_map< int, Eigen::Vector3d > _points;
        std::vector< std::vector<Eigen::Vector3d> > tris_3d;
        std::vector< std::pair< double, std::vector< Eigen::Vector2d > > > tris_pixels;
        std::unordered_map< double, cv::Mat > imgs;

        std::vector< std::vector<Eigen::Vector3d> > tris_3d_single;
        std::vector< std::vector< Eigen::Vector2d >> tris_pixels_single;
        cv::Mat single_img;

        {
            auto it_data = img_dataset.find( show_id );
            if( it_data != img_dataset.end() )
            {
                _img = it_data->second;
            }
        }
        {
            auto it_data = Twcs_dataset.find( show_id );
            if( it_data != Twcs_dataset.end() )
            {
                Twcs_cur = it_data->second;
            }
        }
        {
            auto it_data = points_dataset.find( show_id );
            if( it_data != points_dataset.end() )
            {
                _points = it_data->second;
            }
        }
        {
            auto it_data = Timess_dataset.find(show_id);
            if( it_data != Timess_dataset.end() )
            {
                for( auto time:it_data->second )
                {
                    imgs.insert( {time, img_dataset[ it_data->first ].clone()} );
                }
                single_img = img_dataset[ show_id ].clone();
            }
        }
        {
            auto it_data = tris_dataset.find(show_id);
            auto it_point_set = points_dataset.find(show_id);
            if(it_data!=tris_dataset.end())
            {
                for( const auto & tri : it_data->second.tris)
                {
                    std::vector< Eigen::Vector3d > tri_3d;
                    auto feature_a = tri.second.feature_a;
                    auto feature_b = tri.second.feature_b;
                    auto feature_c = tri.second.feature_c;
                    auto point_a_3d = it_point_set->second.find( feature_a )->second;
                    auto point_b_3d = it_point_set->second.find( feature_b )->second;
                    auto point_c_3d = it_point_set->second.find( feature_c )->second;
                    tri_3d.push_back(point_a_3d);
                    tri_3d.push_back(point_b_3d);
                    tri_3d.push_back(point_c_3d);

                    auto time = tri.second.tri_per_frame.cbegin()->first;
                    auto pixel_a = tri.second.tri_per_frame.cbegin()->second.pixel_a;
                    auto pixel_b = tri.second.tri_per_frame.cbegin()->second.pixel_b;
                    auto pixel_c = tri.second.tri_per_frame.cbegin()->second.pixel_c;

                    std::vector< Eigen::Vector2d > pixels;
                    pixels.push_back( pixel_a ); pixels.push_back( pixel_b ); pixels.push_back( pixel_c );
                    tris_pixels.emplace_back(time, pixels );
                    tris_3d.push_back(tri_3d);

                    {
                        double last_frame_time = Timess_dataset[show_id].back();
                        auto it_pixel_frame = tri.second.tri_per_frame.find( last_frame_time );
                        if( it_pixel_frame != tri.second.tri_per_frame.end() )
                        {
                            auto pixel_a_single = tri.second.tri_per_frame.at( last_frame_time ).pixel_a;
                            auto pixel_b_single = tri.second.tri_per_frame.at( last_frame_time ).pixel_b;
                            auto pixel_c_single = tri.second.tri_per_frame.at( last_frame_time ).pixel_c;

                            std::vector< Eigen::Vector2d > pixels_single;
                            pixels_single.push_back( pixel_a_single ); pixels_single.push_back( pixel_b_single ); pixels_single.push_back( pixel_c_single );
                            tris_pixels_single.push_back( pixels_single );
                            tris_3d_single.push_back(tri_3d);
                        }
                    }

                }
            }
        }

        {
            if( tris_3d_single.empty() )
            {
                if( !tris_3d_single_old.empty() )
                {
                    tris_3d_single = tris_3d_single_old;
                    tris_pixels_single = tris_pixels_single_old;
                    single_img = single_img_old.clone();
                }
            }
            else
            {
                tris_3d_single_old = tris_3d_single;
                tris_pixels_single_old = tris_pixels_single;
                single_img_old = single_img.clone();
            }
        }

        if(menuShowCamera)
            draw_cams( Twcs_cur );

        if(menuShowPoint)
            draw_points(_points);

        if(menuShowMesh)
            draw_mesh(tris_3d, tris_pixels, imgs);

        if(menuShowSingleMesh)
            draw_single_mesh(tris_3d_single, tris_pixels_single, single_img);


        if(menuShowimg)
        {
            std::unique_lock<std::mutex> lock(gui_mutex);
            imageTexture.Upload(_img.data,GL_BGR,GL_UNSIGNED_BYTE);
            d_video.Activate();
            glColor3f(1.0,1.0,1.0);
            // 注意，这里由于Upload函数无法将cv::Mat格式的图片数据作为输入，因此使用 opencv 的data函数将Mat格式的数据变为uchar格式，但是opencv中Mat存储图片是自下而上的，单纯的渲染所渲染出来的图片是倒置的，因此需使用RenderToViewportFlipY（）函数进行渲染，将原本上下倒置的图片进行自下而上渲染，使显示的图片是正的。
            // https://blog.csdn.net/u013270341/article/details/74995530
            imageTexture.RenderToViewportFlipY();
        }

        pangolin::FinishFrame();
    }
}

Eigen::Vector4d pi_f_3p(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3)
{
    Eigen::Vector4d pi;
    Eigen::Vector3d n = ( x1 - x3 ).cross( x2 - x3 );
    n.normalize();
    if(n(2) < 0)
    {
        n(0) = -n(0);
        n(1) = -n(1);
        n(2) = -n(2);
    }
    pi << n, - x3.dot( n ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return pi;
}

GLuint load_texture(const cv::Mat &img)
{
    //OpenGL纹理用整型数表示
    GLuint texture_ID;

    int width = img.cols;
    int height = img.rows;
    //获取图像指针
    int pixellength = width*height * 3;
    auto pixels = new GLubyte[pixellength];
    memcpy(pixels, img.data, pixellength * sizeof(char));
//    imshow("OpenCV", I);

    //将texture_ID设置为2D纹理信息
    glGenTextures(1, &texture_ID);
    glBindTexture(GL_TEXTURE_2D, texture_ID);
    //纹理放大缩小使用线性插值   GL_NEAREST
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //纹理水平竖直方向外扩使用重复贴图
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    //纹理水平竖直方向外扩使用边缘像素贴图(与重复贴图二选一)
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    //将图像内存用作纹理信息
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);

    free(pixels);
    return texture_ID;
}

void Gui::draw_mesh(const std::vector<std::vector<Eigen::Vector3d>> &tris_3d,
                    const std::vector< std::pair< double, std::vector< Eigen::Vector2d >>> &tris_pixels,
                    const std::unordered_map< double, cv::Mat > &imgs)
{
    if(tris_3d.empty())
        return;

    std::map< int64_t, int > datasetid_id;
    cv::Mat img;
    int index_img = 0;
    double img_col_single;
    for(const auto&time_img:imgs)
    {
        if( index_img == 0 )
        {
            img = time_img.second.clone();
            img_col_single = img.cols;
        }
        else
        {
            cv::Mat img_2 = time_img.second.clone();
            cv::hconcat(img,  img_2, img);
        }
        datasetid_id.insert( {time_img.first, index_img} );
        index_img ++;
    }

//    cv::imshow( "img", img );
//    cv::waitKey(1);

//    return;

    GLuint image = load_texture(img);
    double col = img.cols;
    double row = img.rows;
//
    glColor3f(1.0f,1.0f,1.0f);
    //显示纹理
    glEnable(GL_TEXTURE_2D);    //允许使用纹理
    glBindTexture(GL_TEXTURE_2D, image);    //选择纹理对象

    for(int i=0;i<tris_3d.size(); ++i)
//    for(const auto&tri_3d:tris_3d)
    {
        const auto &tri_3d = tris_3d[i];
        const auto &tri_pixels = tris_pixels[i];



        //原始完全填充四边形
        glBegin(GL_TRIANGLES);    //设置为多边形纹理贴图方式并开始贴图
        glTexCoord2f((tri_pixels.second[0][0] + datasetid_id[tri_pixels.first] * img_col_single
        )/col, tri_pixels.second[0][1]/row);
        glVertex3f(tri_3d[0][0], tri_3d[0][1], tri_3d[0][2]);    //纹理左上角对应窗口左上角

        glTexCoord2f((tri_pixels.second[1][0] + datasetid_id[tri_pixels.first] * img_col_single
        )/col, tri_pixels.second[1][1]/row);
        glVertex3f(tri_3d[1][0], tri_3d[1][1], tri_3d[1][2]);    //纹理左下角对应窗口左下角


        glTexCoord2f((tri_pixels.second[2][0] + datasetid_id[tri_pixels.first] * img_col_single
                     )/col, tri_pixels.second[2][1]/row);
        glVertex3f(tri_3d[2][0], tri_3d[2][1], tri_3d[2][2]);    //纹理右下角对应窗口右下角

        glEnd();    //结束贴图*/





//        Eigen::Vector3d norm(0,0,1);
//        Eigen::Vector4d pi = pi_f_3p(tri_3d[0], tri_3d[1], tri_3d[2]);
//        Eigen::Vector3d n = pi.head(3);
//        double o = std::acos( n.dot(norm) );
//        o = o / 3.141592653 ;
//
////        std::cout << o << std::endl;
//
//        glBegin(GL_TRIANGLES);
//        double color = 0.25 + 0.7 * o;
//        glColor3f(color, color, color);
//
//
//        glVertex3f(tri_3d[0][0], tri_3d[0][1], tri_3d[0][2]);
//        glVertex3f(tri_3d[1][0], tri_3d[1][1], tri_3d[1][2]);
//        glVertex3f(tri_3d[2][0], tri_3d[2][1], tri_3d[2][2]);
//
//        glEnd();
    }

    glDisable(GL_TEXTURE_2D);    //禁止使用纹理
    glDeleteTextures(1, &image);
}

void Gui::draw_single_mesh(const std::vector< std::vector< Eigen::Vector3d >> &tris_3d,
                           const std::vector< std::vector< Eigen::Vector2d >> &tris_pixels,
                           const cv::Mat &imgs)
{
    if(tris_3d.empty())
    {
//        std::cout << "tris_3d.empty()" << std::endl;
        return;
    }

    cv::Mat img = imgs.clone();

    GLuint image = load_texture(img);
    double col = img.cols;
    double row = img.rows;
//
    glColor3f(1.0f,1.0f,1.0f);
    //显示纹理
    glEnable(GL_TEXTURE_2D);    //允许使用纹理
    glBindTexture(GL_TEXTURE_2D, image);    //选择纹理对象

    for(int i=0;i<tris_3d.size(); ++i)
//    for(const auto&tri_3d:tris_3d)
    {
        const auto &tri_3d = tris_3d[i];
        const auto &tri_pixels = tris_pixels[i];



        //原始完全填充四边形
        glBegin(GL_TRIANGLES);    //设置为多边形纹理贴图方式并开始贴图
        glTexCoord2f(tri_pixels[0][0]/col, tri_pixels[0][1]/row);
        glVertex3f(tri_3d[0][0], tri_3d[0][1], tri_3d[0][2]);    //纹理左上角对应窗口左上角

        glTexCoord2f(tri_pixels[1][0] /col, tri_pixels[1][1]/row);
        glVertex3f(tri_3d[1][0], tri_3d[1][1], tri_3d[1][2]);    //纹理左下角对应窗口左下角


        glTexCoord2f(tri_pixels[2][0] /col, tri_pixels[2][1]/row);
        glVertex3f(tri_3d[2][0], tri_3d[2][1], tri_3d[2][2]);    //纹理右下角对应窗口右下角

        glEnd();    //结束贴图*/





//        Eigen::Vector3d norm(0,0,1);
//        Eigen::Vector4d pi = pi_f_3p(tri_3d[0], tri_3d[1], tri_3d[2]);
//        Eigen::Vector3d n = pi.head(3);
//        double o = std::acos( n.dot(norm) );
//        o = o / 3.141592653 ;
//
////        std::cout << o << std::endl;
//
//        glBegin(GL_TRIANGLES);
//        double color = 0.25 + 0.7 * o;
//        glColor3f(color, color, color);
//
//
//        glVertex3f(tri_3d[0][0], tri_3d[0][1], tri_3d[0][2]);
//        glVertex3f(tri_3d[1][0], tri_3d[1][1], tri_3d[1][2]);
//        glVertex3f(tri_3d[2][0], tri_3d[2][1], tri_3d[2][2]);
//
//        glEnd();
    }

    glDisable(GL_TEXTURE_2D);    //禁止使用纹理
    glDeleteTextures(1, &image);
}

void Gui::draw_points(const std::unordered_map< int, Eigen::Vector3d > &id_points)
{
    if(id_points.empty())
        return;

    // for ReferenceMapPoints
    //显示局部地图点，大小为4个像素，红色
    glPointSize(8);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(const auto &id_point:id_points)
    {
        const auto &point = id_point.second;
        glVertex3d(point[0], point[1], point[2]);
    }
    glEnd();
}

inline void Gui::draw_cam(const pangolin::OpenGlMatrix &Twc)
{
    //相机模型大小：宽度占总宽度比例为0.08
    const float &w = 0.3;
    const float h = w*0.638;
    const float z = w*0.6;

    //百度搜索：glPushMatrix 百度百科
    glPushMatrix();

    //将4*4的矩阵Twc.m右乘一个当前矩阵
    //（由于使用了glPushMatrix函数，因此当前帧矩阵为世界坐标系下的单位矩阵）
    //因为OpenGL中的矩阵为列优先存储，因此实际为Tcw，即相机在世界坐标下的位姿
    glMultMatrixd(Twc.m);

    //    设置绘制图形时线的宽度
    glLineWidth(1.0);
    //设置当前颜色为绿色(相机图标显示为绿色)
    glColor3f(0.0f,1.0f,0.0f);
    //用线将下面的顶点两两相连
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();


    //双缓存交换缓存以显示图像
    //    glutSwapBuffers();

    glPopMatrix();
}

void Gui::draw_cams( const std::vector< pangolin::OpenGlMatrix > &_Twcs )
{
//    std::unique_lock<std::mutex> lock(gui_mutex);
//        std::unique_lock<std::mutex> lock(gui_mutex, std::chrono::seconds(1));
    if(_Twcs.empty()) return;

    for(const auto &Twc:_Twcs)
    {
        draw_cam(Twc);
    }
}

inline pangolin::OpenGlMatrix GetOpenGlMatrix( const Eigen::Matrix4d &_Twc )
{
    pangolin::OpenGlMatrix M;

    Eigen::Matrix3d Rwc = _Twc.block(0,0,3,3);
    Eigen::Vector3d twc = _Twc.block(0,3,3,1);

    M.m[0] = Rwc(0,0);
    M.m[1] = Rwc(1,0);
    M.m[2] = Rwc(2,0);
    M.m[3]  = 0.0;

    M.m[4] = Rwc(0,1);
    M.m[5] = Rwc(1,1);
    M.m[6] = Rwc(2,1);
    M.m[7]  = 0.0;

    M.m[8] = Rwc(0,2);
    M.m[9] = Rwc(1,2);
    M.m[10] = Rwc(2,2);
    M.m[11]  = 0.0;

    M.m[12] = twc(0);
    M.m[13] = twc(1);
    M.m[14] = twc(2);
    M.m[15]  = 1.0;

    return M;
}

void Gui::update_data(cv::Mat &_img, std::vector<Eigen::Matrix4d> &_Twcs,
                      std::unordered_map< int, Eigen::Vector3d > &id_point_datasets,
//                      std::vector<Eigen::Vector3d> &_Points,
                      TriManager &gui_tri,
                      std::vector< double > &_times,
                      double _time)
{
    std::unique_lock<std::mutex> lock(gui_mutex);

    int64_t dataset_t = Twcs_dataset.size();

    std::vector<pangolin::OpenGlMatrix> Twcs;
    for( const auto &Twc : _Twcs )
    {
        auto Twc_gl = GetOpenGlMatrix( Twc );
        Twcs.push_back( Twc_gl );
    }
    Twcs_dataset.insert( {dataset_t, Twcs} );

    points_dataset.insert( {dataset_t, id_point_datasets} );

    Timess_dataset.insert( {dataset_t, _times} );

    img_dataset.insert( {dataset_t, _img} );

    time_dataset.insert( {dataset_t, _time} );

    time2datasett.insert( {_time, dataset_t} );

    tris_dataset.insert( {dataset_t, gui_tri} );

//    show_id = Twcs_dataset.size() - 1;
}