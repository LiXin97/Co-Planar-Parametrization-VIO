//
// Created by xin on 7/9/21.
//

#include "gui.hpp"

Gui::Gui()
{
    auto proc_func = [&]{run();};
    view_thread.reset( new std::thread(proc_func) );
    show_id = 0;
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
        }
        else
        {
            menuShowframe.Meta().gui_changed = true;

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
        std::vector< Eigen::Vector3d > _points;

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


        if(menuShowCamera)
            draw_cams( Twcs_cur );

        if(menuShowPoint)
            draw_points(_points);

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

void Gui::draw_points(const std::vector<Eigen::Vector3d> &_points)
{
    if(_points.empty())
        return;

    // for ReferenceMapPoints
    //显示局部地图点，大小为4个像素，红色
    glPointSize(8);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
    for(auto point:_points)
    {
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

void Gui::update_data(cv::Mat &_img, std::vector<Eigen::Matrix4d> &_Twcs, std::vector<Eigen::Vector3d> &_Points)
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

    points_dataset.insert( {dataset_t, _Points} );

    img_dataset.insert( {dataset_t, _img} );

//    show_id = Twcs_dataset.size() - 1;
}