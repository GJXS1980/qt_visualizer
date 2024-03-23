#include "window.h"

#include <QMouseEvent>
#include <QPainter>
#include <QTime>
#include <QtMath>

Window::Window(QWidget* parent)
	: QOpenGLWidget(parent)
	, cube(0)
	, texture(0)
	, angularSpeed(0)
{
	fps = 60.0;
}
Window::~Window()
{
    makeCurrent();  // 设置当前OpenGL环境
    delete texture; // 删除纹理对象
    delete cube;    // 删除立方体对象
    doneCurrent();  // 完成当前OpenGL环境
}
void Window::mousePressEvent(QMouseEvent* e)
{
    // 保存鼠标按下的位置
	mousePressPosition = QVector2D(e->localPos());
}
void Window::mouseReleaseEvent(QMouseEvent* e)
{
    // 鼠标释放位置减去鼠标按下位置
	QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;

    //	旋转轴是垂直于鼠标位置差异向量的单位向量
	QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

    // 根据鼠标扫过的长度加速角速度
	qreal acc = diff.length() / 100.0;

    // 计算新的旋转轴作为加权和
	rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();

    // 增加角速度
	angularSpeed += acc;
}

void Window::timerEvent(QTimerEvent*)
{
    // 减小角速度（摩擦）
	angularSpeed *= 0.99;

    // 当速度低于阈值时停止旋转
	if (angularSpeed < 0.01)
	{
		angularSpeed = 0.0;
	}
	else
	{
        // 更新旋转
		rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;
        // 请求更新
		update();
	}
}

void Window::initializeGL()
{
    initializeOpenGLFunctions();    // 初始化OpenGL函数

    glClearColor(0, 0, 0, 1);   // 设置清屏颜色为黑色

    initShaders();  // 初始化着色器
    initTextures(); // 初始化纹理

	//	glEnable(GL_DEPTH_TEST);

    glEnable(GL_CULL_FACE); // 启用面剔除

    cube = new Cube;    // 创建立方体对象

    startTimer(12);     // 启动定时器，每12毫秒触发一次定时器事件
	//	timer.start(12, this);
}


void Window::initShaders()
{
	if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shader/vshader.glsl"))
	{
        qDebug() << __FILE__ << __FUNCTION__ << " 加载顶点着色器文件失败。";
		close();
	}
	if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shader/fshader.glsl"))
	{
        qDebug() << __FILE__ << __FUNCTION__ << " 加载片段着色器文件失败。";
		close();
	}

	if (!program.link())
	{
        qDebug() << __FILE__ << __LINE__ << "程序链接失败";
		close();
	}
	if (!program.bind())
	{
        qDebug() << __FILE__ << __LINE__ << "程序绑定失败";
		close();
	}
}

void Window::initTextures()
{
    texture = new QOpenGLTexture(QImage(":/image/cube.png").mirrored());    // 加载纹理图像并创建纹理对象
    texture->setMinificationFilter(QOpenGLTexture::Nearest);    // 设置缩小过滤器
    texture->setMagnificationFilter(QOpenGLTexture::Linear);    // 设置放大过滤器
    texture->setWrapMode(QOpenGLTexture::Repeat);   // 设置纹理环绕模式
}

void Window::resizeGL(int w, int h)
{
    // 计算宽高比
	qreal aspect = qreal(w) / qreal(h ? h : 1);

    // 设置近平面为3.0，远平面为7.0，视野角度为45度
	const qreal zNear = 3.0, zFar = 7.0, fov = 45.0;

    // 重置投影矩阵
	projection.setToIdentity();
    // 设置透视投影
	projection.perspective(fov, aspect, zNear, zFar);
}

void Window::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);   // 清除颜色缓冲区
	//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    texture->bind();    // 绑定纹理

	QMatrix4x4 matrix;
    matrix.translate(0.0, 0.0, -5.0);   // 平移变换
    matrix.rotate(rotation);    // 旋转变换

    program.setUniformValue("mvp_matrix", projection * matrix); // 设置MVP矩阵

    program.setUniformValue("texture", 0);  // 设置纹理单元

    cube->drawCube(&program);   // 绘制立方体

    calcFPS();  // 计算帧率
    paintFPS(); // 绘制帧率
}

void Window::calcFPS()
{
	static QTime time;
    // 仅执行一次的lambda表达式，用于初始化
	static int	 once = [=]() {
          time.start(); // 启动计时器
		  return 0;
	}();
    Q_UNUSED(once)  // 避免编译器警告
    static int frame = 0;   // 帧计数器
    // 每100帧更新一次帧率
	if (frame++ > 100)
	{
        qreal elasped = time.elapsed(); // 获取经过的时间
        updateFPS(frame / elasped * 1000);  // 更新帧率
        time.restart(); // 重启计时器
        frame = 0;  // 重置帧计数器
	}
}
void Window::updateFPS(qreal v)
{
    fps = v; // 更新帧率值
}
void Window::paintFPS()
{
	//    QString str = QString("FPS:%1").arg(QString::number(fps, 'f', 3));
	//    this->setWindowTitle(str);
}



